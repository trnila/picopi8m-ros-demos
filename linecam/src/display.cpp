#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"

cv::Mat image(128, 128, CV_8U);
const char *window_name = "cv: linecam0";

void chatterCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
    // shift image to the top
    for(int row = 1; row < image.rows; row++) {
        for(int col = 0; col < image.cols; col++) {
            image.at<uint8_t>(row - 1, col) = image.at<uint8_t>(row, col);
        }
    }

    // place incomming row
    int pixels = std::min(image.cols, (int) msg->data.size());
    for(int i = 0; i < pixels; i++) {
        image.at<uint8_t>(image.rows - 1, i) = msg->data[i];
    }
    cv::imshow(window_name, image);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "linecam_display");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/linecam0", 1000, chatterCallback);

    cv::imshow(window_name, image);
    cv::waitKey(1);

    ros::spin();
    return 0;
}
