/**
 * @author TravisHC
 * @brief Trying to migrate the wavefront algorithm in ros2/nav2 to the ros1 interface.
 *        Basically the same with Dijkstra, but it's actually called wavefront. Just practice.
 * @date 2023-09-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _WAVEFRONT_H
#define _WAVEFRONT_H

// priority buffers
#define PRIORITYBUFSIZE 10000
// cost defs
#define COST_UNKNOWN_ROS 255  // 255 is unknown cost
#define COST_OBS 254          // 254 for forbidden regions
#define COST_OBS_ROS 253      // ROS values of 253 are obstacles

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <global_planner/planner_core.h>
#include <global_planner/expander.h>

// clang-format off
// inserting onto the priority blocks
#define push_cur2(n)  { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ && currSize_<PRIORITYBUFSIZE){ currBfr_[currSize_++]=n; pending_[n]=true; }}
#define push_next2(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    nextSize_<PRIORITYBUFSIZE){    nextBfr_[   nextSize_++]=n; pending_[n]=true; }}
#define push_over2(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    overSize_<PRIORITYBUFSIZE){    overBfr_[   overSize_++]=n; pending_[n]=true; }}
// clang-format on

namespace global_planner {
class WavefrontExpansion : public Expander {
public:
    WavefrontExpansion(PotentialCalculator* p_calc, int nx, int ny);
    ~WavefrontExpansion();
    void setSize(int nx, int ny); /**< sets or resets the size of the map */

    void setNeutralCost(unsigned char neutral_cost) {
        neutral_cost_      = neutral_cost;
        priorityIncrement_ = 2 * neutral_cost_;
    }

    void setPreciseStart(bool precise) {
        precise_ = precise;
    }

    void updateCell(unsigned char* costs, float* potential, int n); /** updates the cell at index n */

    float getCost(unsigned char* costs, int n) {
        float c = costs[n];
        if (c < lethal_cost_ - 1 || (unknown_ && c == 255)) {
            c = c * factor_ + neutral_cost_;
            if (c >= lethal_cost_)
                c = lethal_cost_ - 1;
            return c;
        }
        return lethal_cost_;
    }

    bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles, float* potential);

    /** block priority buffers */
    int *pb1_, *pb2_, *pb3_;             /**< storage buffers for priority blocks */
    int *currBfr_, *nextBfr_, *overBfr_; /**< priority buffer block ptrs */
    int currSize_, nextSize_, overSize_; /**< end points of arrays */
    bool* pending_;                      /**< pending_ cells during propagation */
    bool precise_;

    /** block priority thresholds */
    float threshold_;         /**< current threshold */
    float priorityIncrement_; /**< priority threshold increment */
};
}  // namespace global_planner
#endif