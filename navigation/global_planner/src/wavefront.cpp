#include <global_planner/wavefront.h>
#include <algorithm>

namespace global_planner {

WavefrontExpansion::WavefrontExpansion(PotentialCalculator* p_calc, int nx, int ny)
    : Expander(p_calc, nx, ny), pending_(NULL), precise_(false) {
    // priority buffers
    pb1_ = new int[PRIORITYBUFSIZE];
    pb2_ = new int[PRIORITYBUFSIZE];
    pb3_ = new int[PRIORITYBUFSIZE];

    priorityIncrement_ = 2 * neutral_cost_;
}

WavefrontExpansion::~WavefrontExpansion() {
    delete[] pb1_;
    delete[] pb2_;
    delete[] pb3_;
    if (pending_)
        delete[] pending_;
}

void WavefrontExpansion::setSize(int xs, int ys) {
    Expander::setSize(xs, ys);
    if (pending_)
        delete[] pending_;

    pending_ = new bool[ns_];
    memset(pending_, 0, ns_ * sizeof(bool));
}

bool WavefrontExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles, float* potential) {
    cells_visited_ = 0;
    // priority buffers
    threshold_ = lethal_cost_;
    currBfr_   = pb1_;
    currSize_  = 0;
    nextBfr_   = pb2_;
    nextSize_  = 0;
    overBfr_   = pb3_;
    overSize_  = 0;
    memset(pending_, 0, ns_ * sizeof(bool));
    std::fill(potential, potential + ns_, POT_HIGH);

    // outer bounds of cost array
    unsigned char* pc;
    pc = costs;
    for (int i = 0; i < nx_; i++) {
        *pc++ = COST_OBS;
    }
    pc = costs + (ny_ - 1) * nx_;
    for (int i = 0; i < nx_; i++) {
        *pc++ = COST_OBS;
    }
    pc = costs;
    for (int i = 0; i < ny_; i++, pc += nx_) {
        *pc = COST_OBS;
    }
    pc = costs + nx_ - 1;
    for (int i = 0; i < ny_; i++, pc += nx_) {
        *pc = COST_OBS;
    }

    // set goal
    int k = toIndex(start_x, start_y);

    //  当前点的代价值由potential数组维护，push_cur()函数将当前点的邻节点push进去，当前遍历的邻节点用currBfr_数组维护，currentSize_记录该数组的长度？ TODO

    if (precise_) {
        /* c为当前点，o为push的邻节点, precise_为true时，将如下所示的8个邻节点都push进去
              o o
            o c c o
            o c c o
              o o
        */

        double dx = start_x - (int)start_x, dy = start_y - (int)start_y;
        dx                     = floorf(dx * 100 + 0.5) / 100;
        dy                     = floorf(dy * 100 + 0.5) / 100;
        potential[k]           = neutral_cost_ * 2 * dx * dy;
        potential[k + 1]       = neutral_cost_ * 2 * (1 - dx) * dy;
        potential[k + nx_]     = neutral_cost_ * 2 * dx * (1 - dy);
        potential[k + nx_ + 1] = neutral_cost_ * 2 * (1 - dx) * (1 - dy);  //*/

        // push_cur除了边界判断外其实就做了两步操作：1.用currBfr_记录当前有效的邻节点，并将当前currentSize_++；2.将当前邻节点的pending_标记为true
        push_cur2(k + 2);
        push_cur2(k - 1);
        push_cur2(k + nx_ - 1);
        push_cur2(k + nx_ + 2);

        push_cur2(k - nx_);
        push_cur2(k - nx_ + 1);
        push_cur2(k + nx_ * 2);
        push_cur2(k + nx_ * 2 + 1);
    } else {
        /* c为当前点，o为push的邻节点，precise_为false时，只是简单地将上下左右4个邻节点push进去
              o
            o c o
              o
        */
        potential[k] = 0;
        push_cur2(k + 1);
        push_cur2(k - 1);
        push_cur2(k - nx_);
        push_cur2(k + nx_);
    }

    int max   = 0;  // max priority block size
    int num   = 0;  // number of cells put into priority blocks
    int cycle = 0;  // which cycle we're on

    //  从终点开始建立势场
    int startCell = toIndex(end_x, end_y);
    for (; cycle < cycles; cycle++) {
        if (currSize_ == 0 && nextSize_ == 0) return false;

        num += currSize_;
        max = currSize_ > max ? currSize_ : max;

        int* pb = currBfr_;
        int i   = currSize_;
        while (i-- > 0) {
            pending_[*(pb++)] = false;
        }

        pb = currBfr_;
        i  = currSize_;
        while (i--) {
            updateCell(costs, potential, *pb++);
        }

        currSize_ = nextSize_;
        nextSize_ = 0;
        pb        = currBfr_;
        currBfr_  = nextBfr_;
        nextBfr_  = pb;

        if (currSize_ == 0) {
            threshold_ += priorityIncrement_;
            currSize_ = overSize_;
            overSize_ = 0;
            pb        = currBfr_;
            currBfr_  = overBfr_;
            overBfr_  = pb;
        }

        if (potential[startCell] < POT_HIGH) {
            break;
        }
    }

    return cycle < cycles ? true : false;
}

//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void WavefrontExpansion::updateCell(unsigned char* costs, float* potential, int n) {
    cells_visited_++;

    // do planar wave update
    float c = getCost(costs, n);
    if (c >= lethal_cost_)  // don't propagate into obstacles
        return;

    float pot = p_calc_->calculatePotential(potential, c, n);

    // now add affected neighbors to priority blocks
    if (pot < potential[n]) {
        float le     = INVSQRT2 * (float)getCost(costs, n - 1);
        float re     = INVSQRT2 * (float)getCost(costs, n + 1);
        float ue     = INVSQRT2 * (float)getCost(costs, n - nx_);
        float de     = INVSQRT2 * (float)getCost(costs, n + nx_);
        potential[n] = pot;
        // ROS_INFO("UPDATE %d %d %d %f", n, n%nx, n/nx, potential[n]);
        if (pot < threshold_)  // low-cost buffer block
        {
            if (potential[n - 1] > pot + le)
                push_next2(n - 1);
            if (potential[n + 1] > pot + re)
                push_next2(n + 1);
            if (potential[n - nx_] > pot + ue)
                push_next2(n - nx_);
            if (potential[n + nx_] > pot + de)
                push_next2(n + nx_);
        } else  // overflow block
        {
            if (potential[n - 1] > pot + le)
                push_over2(n - 1);
            if (potential[n + 1] > pot + re)
                push_over2(n + 1);
            if (potential[n - nx_] > pot + ue)
                push_over2(n - nx_);
            if (potential[n + nx_] > pot + de)
                push_over2(n + nx_);
        }
    }
}

}  // namespace global_planner