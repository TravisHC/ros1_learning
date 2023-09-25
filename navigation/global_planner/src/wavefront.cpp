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

WavefrontExpansion::setSize(int xs, int ys) {
    Expander::setSize(xs, ys);
    if (pending_)
        delete[] pending_;

    pending_ = new bool[ns_];
    memset(pending_, 0, ns_ * sizeof(bool));
}

// clang-format off
// inserting onto the priority blocks
#define push_cur(n)  { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ && currentEnd_<PRIORITYBUFSIZE){ currentBuffer_[currentEnd_++]=n; pending_[n]=true; }}
#define push_next(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    nextEnd_<PRIORITYBUFSIZE){    nextBuffer_[   nextEnd_++]=n; pending_[n]=true; }}
#define push_over(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    overEnd_<PRIORITYBUFSIZE){    overBuffer_[   overEnd_++]=n; pending_[n]=true; }}
// clang-format on

WavefrontExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles, float* potential) {
    // reset values in propagation arrays
    memset(potential, POT_HIGH, ns_ * sizeof(float));

    // outer bounds of cost array
    COSTTYPE* pc;
    pc = costarr;
    for (int i = 0; i < nx; i++) {
        *pc++ = COST_OBS;
    }
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++) {
        *pc++ = COST_OBS;
    }
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx) {
        *pc = COST_OBS;
    }
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx) {
        *pc = COST_OBS;
    }
}

}  // namespace global_planner