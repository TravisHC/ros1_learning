#include "astar_hybrid.h"

namespace ros1_smac_planner {

HybridAStarExpansion::HybridAStarExpansion(PotentialCalculator *p_calc, int xs, int ys)
    : Expander(p_calc, xs, ys) {
}


}  // namespace global_planner
