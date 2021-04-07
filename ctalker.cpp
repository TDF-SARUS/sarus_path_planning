#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <iostream>
#include <vector>
#include <array>

#define H (4)
#define W (5)

int fmission(float **array)
{
    int argc; char **argv;
    ros::init(argc, argv, "ctalker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("matrix_pub", 1);
    ros::Rate loop_rate(10);
    std_msgs::Float32MultiArray dat;

    // fill out message:
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim[0].label = "height";
    dat.layout.dim[1].label = "width";
    dat.layout.dim[0].size = H;
    dat.layout.dim[1].size = W;
    dat.layout.dim[0].stride = H*W;
    dat.layout.dim[1].stride = W;
    dat.layout.data_offset = 0;
    std::vector<float> vec(W*H, 0);
    for (int i=0; i<H; i++)
        for (int j=0; j<W; j++)
            vec[i*W + j] = array[i][j];
    dat.data = vec;

    while (ros::ok())
    {
        pub.publish(dat);
	ROS_INFO("I published something!");
	ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

