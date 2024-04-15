#ifndef TF_SOLUTION_H
#define TF_SOLUTION_H

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <string>

class TFSolution {
protected:
    //You should use a  tf2_ros::Buffer, a tf2_ros::TransformListener and a
    //  tf2_ros::TransformBroadcaster in your implementation of this class
    // These should be scoped to the CLASS, not to a class method, and not 
    //  static
    tf2_ros::TransformBroadcaster _tfBroadcaster;
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;


    void printTransformStamped(geometry_msgs::TransformStamped transformStamped);
    void broadcastTransform(std::string parent, std::string child, Eigen::MatrixXd transform);

    Eigen::MatrixXd getTransformMatrix(geometry_msgs::TransformStamped transformStamped);
    
    Eigen::MatrixXd oneMeterForwardOnX();
    Eigen::MatrixXd oneMeterUpOnZ();
    Eigen::MatrixXd rotate180DegreesOnZ();


public:
    TFSolution();
    ~TFSolution();

    void update();
};

#endif