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

#define PRIORITYBUFSIZE 10000
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <global_planner/planner_core.h>
#include <global_planner/expander.h>

// cost defs
#define COST_UNKNOWN_ROS 255  // 255 is unknown cost
#define COST_OBS 254  // 254 for forbidden regions
#define COST_OBS_ROS 253  // ROS values of 253 are obstacles

// navfn cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.
// Incoming costmap cost values are in the range 0 to 252.
// With COST_NEUTRAL of 50, the COST_FACTOR needs to be about 0.8 to
// ensure the input values are spread evenly over the output range, 50
// to 253.  If COST_FACTOR is higher, cost values will have a plateau
// around obstacles and the planner will then treat (for example) the
// whole width of a narrow hallway as equally undesirable and thus
// will not plan paths down the center.

#define COST_NEUTRAL 50  // Set this to "open space" value
#define COST_FACTOR 0.8  // Used for translating costs in NavFn::setCostmap()

// Define the cost type in the case that it is not set. However, this allows
// clients to modify it without changing the file. Arguably, it is better to require it to
// be defined by a user explicitly
#ifndef COSTTYPE
#define COSTTYPE unsigned char  // Whatever is used...
#endif

// potential defs
#define POT_HIGH 1.0e10  // unassigned cell potential

// priority buffers
#define PRIORITYBUFSIZE 10000

namespace global_planner {
class WavefrontExpansion : public Expander {
public:
    WavefrontExpansion()
}
}  // namespace global_planner