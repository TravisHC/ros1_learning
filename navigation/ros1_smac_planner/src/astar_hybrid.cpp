#include "astar_hybrid.h"

namespace global_planner {
HybridAStarExpansion::HybridAStarExpansion(PotentialCalculator *p_calc, int xs, int ys)
    : Expander(p_calc, xs, ys) {
}


bool HybridAStarExpansion::calculatePotentials(unsigned char *costs, double start_x, double start_y, double end_x,
                                               double end_y, int cycles, float *potential) {
    // queue_为启发式搜索到的向量队列：<i , cost>
    queue_.clear();
    // 起点的索引
    int start_i = toIndex(start_x, start_y);
    // step 1 将起点放入队列
    queue_.push_back(Index(start_i, 0));
    // step 2 potential数组值全设为极大值
    // std::fill(a,b,x) 将a到b的元素都赋予x值
    std::fill(potential, potential + ns_, POT_HIGH);  // ns_ : 单元格总数
    // step 3 将起点的potential设为0
    potential[start_i] = 0;
    // 目标索引
    int goal_i = toIndex(end_x, end_y);
    int cycle  = 0;
    // 进入循环，继续循环的判断条件为只要队列大小大于0且循环次数小于所有格子数的2倍
    while (queue_.size() > 0 && cycle < cycles) {
        // step 4 得到最小cost的索引，并删除它，如果索引指向goal(目的地)则退出算法，返回true
        Index top = queue_[0];
        // step 4.1 将向量第一个元素(最小的代价的Index)和向量最后一个位置元素对调，再用pop_back删除这个元素
        // pop_heap(Iter,Iter,_Compare) _Compare有两种参数，一种是greater（小顶堆），一种是less（大顶堆）,先对调，再排序
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        // 删除最小代价的点
        queue_.pop_back();

        int i = top.i;
        // step 4.2 若是目标点则终止搜索，搜索成功
        if (i == goal_i)
            return true;
    }
}


}  // namespace global_planner
