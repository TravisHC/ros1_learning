/**
 * @author TravisHC
 * @brief Migeration of Hybrid A* algorithm from ROS2/nav2/smac_planner
 * @date 2023-10-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

class HybridAStarExpansion : public Expander
{
public:
    HybridAStarExpansion(PotentialCalculator *p_calc, int xs, int ys);
    bool calculatePotentials(unsigned char *costs, double start_x, double start_y, double end_x, double end_y, int cycles, float *potential);
}