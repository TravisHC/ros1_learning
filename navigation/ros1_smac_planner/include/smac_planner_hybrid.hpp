/**
 * @author TravisHC
 * @brief Migeration of Hybrid A* algorithm from ROS2/nav2/smac_planner
 * @date 2023-10-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "base_smac_planner.h"

class HybridAStarExpansion : public BaseSmacPlanner
{
public:
    HybridAStarExpansion(PotentialCalculator *p_calc, int xs, int ys);
}