#include <tf_homework/TFSolution.h>
#include <ros/console.h>
#include <ros/ros.h>

using namespace ros;
using namespace std;
TFSolution::TFSolution() : _tfListener(_tfBuffer), _tfBroadcaster() {}
TFSolution::~TFSolution() {}

Eigen::MatrixXd TFSolution::getTransformMatrix(geometry_msgs::TransformStamped transformStamped) {
    //Use transformStamped to fill in a 4x4 rigid transformation matrix
    //DO NOT USE the ROS package tf2_eigen for your solution.
    //Write your solution in Eigen. Use Eigen::MatrixXd as your 4x4 rigid
    //  transformation
    //Use Eigen::Quaterniseqond to get the rotation matrix portion of the 
    //  rigid transformation.
    Eigen::Matrix4d sol;

    Eigen::Quaterniond r(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
    Eigen::Matrix3d rMat = r.matrix();
    sol.block<3, 3>(0, 0) = rMat;
    sol(0, 3) = transformStamped.transform.translation.x;
    sol(1, 3) = transformStamped.transform.translation.y;
    sol(2, 3) = transformStamped.transform.translation.z;
    sol(3, 3) = 1;
    return sol;
}

Eigen::MatrixXd TFSolution::oneMeterForwardOnX() {
    //Use Eigen::MatrixXd to create a 4x4 rigid transformation matrix that
    //  translates 1 meter forward on the X axis
    Eigen::Matrix4d sol;

    sol(0,0) = 1;
    sol(1,1) = 1;
    sol(2,2) = 1;
    sol(3,3) = 1;
    sol(0,3) = 1;

    return sol;
}

Eigen::MatrixXd TFSolution::oneMeterUpOnZ() {
    //Use Eigen::MatrixXd to create a 4x4 rigid transformation matrix that
    //  translates 1 met er up on the Z axis
    Eigen::Matrix4d sol;

    sol(0,0) = 1;
    sol(1,1) = 1;
    sol(2,2) = 1;
    sol(3,3) = 1;
    sol(2,3) = 1;

    return sol;
}

Eigen::MatrixXd TFSolution::rotate180DegreesOnZ() {
    //Use Eigen::MatrixXd + Eigen::AngleAxisd to create a 4x4 rigid
    //  transformation matrix that rotates 180 degrees about the Z axis
    Eigen::Matrix4d sol;

    sol(0,0) = -1;
    sol(1,1) = -1;
    sol(2,2) = 1;
    sol(3,3) = 1;

    return sol;
    return Eigen::MatrixXd();
}

void TFSolution::broadcastTransform(std::string parent, std::string child, Eigen::MatrixXd transform) {
    //Use Eigen (matrix functions + Eigen::Quaterniond) to fill in the fields
    //  of a geometry_msgs::TransformStamped
    //Then use a tf2_ros::TransformBroadcaster to broadcast the transform
    //Do NOT make a new tf2_ros::TransformBroadcaster every time this function is called
    geometry_msgs::TransformStamped transformStamped;

    Eigen::Quaterniond q(transform.block<3,3>(0,0));
    transformStamped.header.frame_id = parent;
    transformStamped.child_frame_id = child;

    transformStamped.header.stamp = ros::Time::now();
    
    transformStamped.transform.translation.x = transform(0, 3);
    transformStamped.transform.translation.y = transform(1, 3);
    transformStamped.transform.translation.z = transform(2, 3);

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    _tfBroadcaster.sendTransform(transformStamped);
}

void TFSolution::printTransformStamped(geometry_msgs::TransformStamped transformStamped) {
    //Use ROS_INFO_STREAM to print every field in transformStamped
    ROS_INFO_STREAM("--NEW TRANSFORM--" << endl);
    ROS_INFO_STREAM("header.seq: \t\t\t" << transformStamped.header.seq);
    ROS_INFO_STREAM("header.stamp: \t\t\t" << transformStamped.header.stamp);
    ROS_INFO_STREAM("header.frame_id: \t\t" << transformStamped.header.frame_id);
    ROS_INFO_STREAM("child_frame_id: \t\t" << transformStamped.child_frame_id);
    ROS_INFO_STREAM("transform.translation.x: \t" << transformStamped.transform.translation.x);
    ROS_INFO_STREAM("transform.translation.y \t" << transformStamped.transform.translation.y);
    ROS_INFO_STREAM("transform.translation.z: \t" << transformStamped.transform.translation.z);
    ROS_INFO_STREAM("transform.rotation.x: \t\t" << transformStamped.transform.rotation.x);
    ROS_INFO_STREAM("transform.rotation.y: \t\t" << transformStamped.transform.rotation.y);
    ROS_INFO_STREAM("transform.rotation.z: \t\t" << transformStamped.transform.rotation.z);
    ROS_INFO_STREAM("transform.rotation.w: \t\t" << transformStamped.transform.rotation.w);
}

void TFSolution::update() {
    try{
        geometry_msgs::TransformStamped transformStamped;
        //Fill in transformStamped with the transform between fixed_link and
        //  base_link using TF2's lookupTransform.
        transformStamped = _tfBuffer.lookupTransform("fixed_link", "base_link", ros::Time(0));

        //Make it so this prints out the fields of the transformStamped message
        printTransformStamped(transformStamped);

        //Make these broadcastTransform calls broadcast 3 transforms:
        //  forward_x -> One meter forward on the X axis of base_link
        //  forward_x_up_z -> forward_x, then up 1 meter on the Z axis
        //  forward_x_up_z_180z -> forward_x_up_z, then rotated 180 degrees
        //      on the Z axis
        //Note that each of these should be broadcast relative to fixed_link
        Eigen::MatrixXd forward_x = oneMeterForwardOnX();
        Eigen::MatrixXd forward_x_up_z = oneMeterUpOnZ();
        Eigen::MatrixXd forward_x_up_z_180z = rotate180DegreesOnZ();

        broadcastTransform("fixed_link", "forward_x", forward_x);
        broadcastTransform("fixed_link", "forward_x_up_z", forward_x_up_z);
        broadcastTransform("fixed_link", "forward_x_up_z_180z", forward_x_up_z_180z);

    } catch (tf2::TransformException &ex) {
    }
}