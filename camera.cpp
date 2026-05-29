#include "opencv2/opencv.hpp"
#include "opencv2/core/utils/logger.hpp"
extern "C" {
#include <iic.h>
}
#include <cstdio>
#include <iostream>

cv::VideoCapture camera(0);

void camera_init() {
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return;
    }
    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    camera.set(cv::CAP_PROP_EXPOSURE, 1000);
}

void camera_run() {
    cv::Mat frame, hsv, maskWhite, maskBlack, maskRed, maskRed1, maskRed2, maskGreen, maskBlue;
    camera >> frame;
    if (frame.empty()) return;

    double focalLength = 837.0;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar lowerBoundRed1(0, 100, 100);
    cv::Scalar upperBoundRed1(10, 255, 255);
    cv::Scalar lowerBoundRed2(160, 100, 100);
    cv::Scalar upperBoundRed2(180, 255, 255);
    cv::inRange(hsv, lowerBoundRed1, upperBoundRed1, maskRed1);
    cv::inRange(hsv, lowerBoundRed2, upperBoundRed2, maskRed2);
    cv::bitwise_or(maskRed1, maskRed2, maskRed);

    cv::Scalar lowerBoundWhite(140, 0, 200);
    cv::Scalar upperBoundWhite(180, 10, 255);
    cv::inRange(hsv, lowerBoundWhite, upperBoundWhite, maskWhite);

    cv::Scalar lowerBoundBlack(0, 0, 0);
    cv::Scalar upperBoundBlack(180, 255, 100);
    cv::inRange(hsv, lowerBoundBlack, upperBoundBlack, maskBlack);

    cv::Scalar lowerBoundGreen(40, 50, 100);
    cv::Scalar upperBoundGreen(80, 255, 255);
    cv::inRange(hsv, lowerBoundGreen, upperBoundGreen, maskGreen);

    cv::Scalar lowerBoundBlue(100, 50, 100);
    cv::Scalar upperBoundBlue(130, 255, 255);
    cv::inRange(hsv, lowerBoundBlue, upperBoundBlue, maskBlue);

    std::vector<std::pair<std::vector<std::vector<cv::Point>>, std::string>> allContours;
    for (auto& [mask, name] : std::vector<std::pair<cv::Mat*, std::string>>{
        {&maskWhite, "white"}, {&maskBlack, "black"}, {&maskRed, "red"},
        {&maskGreen, "green"}, {&maskBlue, "blue"}})
    {
        cv::erode(*mask, *mask, cv::Mat(), cv::Point(-1,-1), 2);
        cv::dilate(*mask, *mask, cv::Mat(), cv::Point(-1,-1), 2);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(*mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (!contours.empty()) allContours.push_back({contours, name});
    }

    std::cout << "\033[H";
    for (auto& [contours, name] : allContours) {
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) < 1000) continue;
            cv::RotatedRect rotRect = cv::minAreaRect(contour);
            cv::Point2f center = rotRect.center;
            cv::Size2f sz = rotRect.size;
            float angle = rotRect.angle;

            double distanceX = (1-center.y / frame.rows) * 35 + 5;
            double distanceY = 20.0;
            double distance = std::sqrt(distanceX * distanceX + distanceY * distanceY);
            double aspectRatio = sz.width / sz.height;
            if (aspectRatio > 2.0 || aspectRatio < 0.5) continue;

            double rectArea = sz.width * sz.height;
            if (rectArea < 1) continue;
            double solidity = cv::contourArea(contour) / rectArea;
            if (solidity < 0.65) continue;

            double size = std::max(sz.width, sz.height) / (focalLength / distance);

            cv::Point2f corners[4];
            rotRect.points(corners);
            for (int i = 0; i < 4; i++)
                cv::line(frame, corners[i], corners[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);

            std::cout << name << " with width " << size << "cm at distance " << distance << std::endl;
            cv::circle(frame, center, 5, cv::Scalar(0, 0, 255), -1);
            cv::putText(frame, name, cv::Point(center.x - 20, center.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
            cv::putText(frame, std::to_string(size), cv::Point(center.x - 20, center.y - 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        }
    }
    std::cout << "\033[H\033[J";
}
