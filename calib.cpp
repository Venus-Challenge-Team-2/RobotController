#include "opencv2/opencv.hpp"
#include "opencv2/core/utils/logger.hpp"
#include <iostream>

int main() {
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

    system("v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=1");

    cv::VideoCapture camera(2, cv::CAP_V4L2);
    cv::waitKey(500);

    cv::namedWindow("Camera");
    cv::createTrackbar("Gain", "Camera", NULL, 100, [](int pos, void*) {
        std::string cmd = "v4l2-ctl -d /dev/video2 --set-ctrl=gain=" 
                          + std::to_string(pos-50);
        system(cmd.c_str());
    }, nullptr);
    cv::setTrackbarPos("Gain", "Camera", 0); // min gain = darkest

    while (true) {
        cv::Mat frame;
        camera >> frame;
        cv::imshow("Camera", frame);
        if (cv::waitKey(30) == 'q') break;
    }

    return 0;
}
