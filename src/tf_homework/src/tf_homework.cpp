#include <tf_homework/TFSolution.h>

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_homework");
    ros::NodeHandle node;

    TFSolution tfSolution;

    ros::Rate rate(10);

    while (node.ok()){
        tfSolution.update();
        rate.sleep();
    }
}