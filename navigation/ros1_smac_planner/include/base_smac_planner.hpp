/**
 * @author TravisHC
 * @brief inherit from base_global_planner and should be an interface of different implementions of smac planner, also a ros interface with some utils.
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef ROS1_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
#define ROS1_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_

#include <memory>
#include <vector>
#include <string>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#inlcude "nav_core/base_global_planner.h"

namespace ros1_smac_planner {

class BaseSmacPlanner : public nav_core::BaseGlobalPlanner {
public:
    BaseSmacPlanner();
    ~BaseSmacPlanner();

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override {
        config(name, costmap_ros);
    };

    virtual bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) override = 0;

    virtual void config(std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros) = 0;

protected:

inline void mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx + convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my + convert_offset_) * costmap_->getResolution();
}

inline bool worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}



}
}  // namespace ros1_smac_planner