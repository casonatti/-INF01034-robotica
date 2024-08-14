#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#include "Utils.h"

#define OCC_OCCUPIED 100
#define OCC_NEAROBSTACLE 90
#define OCC_FREE 50
#define OCC_FRONTIER 0
#define OCC_UNEXPLORED -1

class Perception
{
public:
    Perception(ros::NodeHandle& n);

    bool hasValidGradientDescent();
    double getDiffAngleToGradientDescent();
    
private:
    void receiveGridmap(const nav_msgs::OccupancyGrid::ConstPtr &value);

    void updateGridKnownLimits();
    void updateCellsClassification();
    void updatePotentialField();
    double computeDirectionOfGradientDescent(Pose2D robotPose);

    Pose2D getCurrentRobotPose();

    bool started_;
    bool validGradientDescent_;

    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener* tfListener_;
    ros::Subscriber sub_gridmap_;
    ros::Publisher pub_mapPotField_, pub_mapBoundaries_, pub_gradientDescent_;

    int numCellsX_;
    int numCellsY_;
    float mapWidth_;
    float mapHeight_;
    float scale_;

    unsigned int minKnownX_, minKnownY_, maxKnownX_, maxKnownY_;

    std::vector<int8_t> boundariesGrid_;
    std::vector<float> potFieldGrid_;
    double diffAngleToGradientDescent_;

    nav_msgs::OccupancyGrid msg_boundaries_;
    nav_msgs::OccupancyGrid msg_potField_;
    geometry_msgs::PoseStamped msg_gradientDescent_;
};

#endif // PERCEPTION_H
