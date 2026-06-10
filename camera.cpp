#include "opencv2/opencv.hpp"
#include "opencv2/core/utils/logger.hpp"
extern "C" {
#include <iic.h>
}
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include "mqtt.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern bool scanning;
cv::VideoCapture camera(0);

float robot_height_cm = 27.5;
float camera_tilt_deg = 30.0; // Angle down from horizontal

cv::Point2f projectToGround(float px, float py, int w, int h, float focalLength, float H, float tiltDeg) {
    float theta = tiltDeg * M_PI / 180.0;
    float x_rel = px - w / 2.0;
    float y_rel = py - h / 2.0;

    float denom = focalLength * sin(theta) + y_rel * cos(theta);
    if (std::abs(denom) < 1e-6) return cv::Point2f(0, 0);

    float ground_forward = H * (focalLength * cos(theta) - y_rel * sin(theta)) / denom;
    float ground_lateral = H * x_rel / denom;

    return cv::Point2f(ground_lateral, ground_forward);
}

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

    std::vector<std::pair<cv::Mat*, std::string>> masks = {
        {&maskWhite, "white"}, {&maskBlack, "black"}, {&maskRed, "red"},
        {&maskGreen, "green"}, {&maskBlue, "blue"}
    };

    std::vector<std::pair<std::vector<std::vector<cv::Point>>, std::string>> allContours;
    for (auto& [mask, name] : masks)
    {
        cv::erode(*mask, *mask, cv::Mat(), cv::Point(-1,-1), 2);
        cv::dilate(*mask, *mask, cv::Mat(), cv::Point(-1,-1), 2);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(*mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (!contours.empty()) allContours.push_back({contours, name});
    }

    //std::cout << "\033[H\033[J";
    std::ostringstream oss;
    for (auto& [contours, name] : allContours) {
        for (const auto& contour : contours) {
            double area_px = cv::contourArea(contour);
            if (area_px < 500) continue;

            cv::RotatedRect rotRect = cv::minAreaRect(contour);
            cv::Point2f center = rotRect.center;

            // Project contour to ground for accurate metrics
            std::vector<cv::Point2f> groundContour;
            for (const auto& p : contour) {
                groundContour.push_back(projectToGround(p.x, p.y, frame.cols, frame.rows, focalLength, robot_height_cm, camera_tilt_deg));
            }
            double area_cm2 = cv::contourArea(groundContour);

            cv::Point2f groundCenter = projectToGround(center.x, center.y, frame.cols, frame.rows, focalLength, robot_height_cm, camera_tilt_deg);
            double x_rel = groundCenter.x;
            double y_rel = groundCenter.y;
            double distance = std::sqrt(x_rel * x_rel + y_rel * y_rel);

            double rw = rotRect.size.width;
            double rh = rotRect.size.height;
            double rectArea = rw * rh;
            if (rectArea < 1) continue;

            double ratio = (rw > rh) ? (rw / rh) : (rh / rw);
            if (ratio > 5.0) continue;

            double solidity = area_px / rectArea;
            if (solidity < 0.6) continue;

            // Estimate size from ground contour bounding box
            cv::Rect2f groundRect = cv::boundingRect(groundContour);
            double size = groundRect.width;

            int colorIdx = -1;
            if (name == "red") colorIdx = 0;
            else if (name == "black") colorIdx = 1;
            else if (name == "blue") colorIdx = 2;
            else if (name == "green") colorIdx = 3;
            else if (name == "white") colorIdx = 4;

            if (name == "black") {
                if (area_cm2 >= 100) {
                    std::cout << "HOLE at (" << (int)x_rel << ", " << (int)y_rel << ") Area: " << (int)area_cm2 << "cm2" << std::endl;
                    std::cout << "X: " <<robot_x << " Y: " << robot_y << std::endl;
                    // Fill the map with hole points (grid where each point is 3x3cm)

                    for (float gy_rel = groundRect.y; gy_rel < groundRect.y + groundRect.height; gy_rel += 3.0f) {
                        for (float gx_rel = groundRect.x; gx_rel < groundRect.x + groundRect.width; gx_rel += 3.0f) {
                            if (cv::pointPolygonTest(groundContour, cv::Point2f(gx_rel, gy_rel), false) >= 0) {
                                int gx = (int)(robot_x + (gx_rel * std::cos(robot_angle) + gy_rel * std::sin(robot_angle)) / 3.0f);
                                int gy = (int)(robot_y + (gy_rel * std::cos(robot_angle) - gx_rel * std::sin(robot_angle)) / 3.0f);
                                if (gx >= 0 && gx < 333 && gy >= 0 && gy < 333) {
                                    oss << "214" << std::setfill('0') << std::setw(3) << gx
                                        << std::setfill('0') << std::setw(3) << gy << "\n";
                                }
                            }
                        }
                    }
                }
            } else {
                int sizeDigit = -1;
                if (size >= 2.0 && size <= 4.0) sizeDigit = 1;
                else if (size >= 5.0 && size <= 7.0) sizeDigit = 2;

                if (sizeDigit != -1 && colorIdx != -1) {
                    int gx = (int)(robot_x + (x_rel * std::cos(robot_angle) + y_rel * std::sin(robot_angle)) / 3.0f);
                    int gy = (int)(robot_y + (y_rel * std::cos(robot_angle) - x_rel * std::sin(robot_angle)) / 3.0f);

                    if (gx >= 0 && gx < 333 && gy >= 0 && gy < 333) {
                        oss << "2" << colorIdx << sizeDigit << std::setfill('0') << std::setw(3) << gx
                            << std::setfill('0') << std::setw(3) << gy << "\n";
                    }
                }
                std::cout << name << " with width " << size << "cm at distance " << distance << std::endl;
            }

            cv::Point2f corners[4];
            rotRect.points(corners);
            for (int i = 0; i < 4; i++)
                cv::line(frame, corners[i], corners[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);

            cv::circle(frame, center, 5, cv::Scalar(0, 0, 255), -1);
            cv::putText(frame, name, cv::Point(center.x - 20, center.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
            cv::putText(frame, std::to_string(size), cv::Point(center.x - 20, center.y - 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        }
    }
    uart_send_string(oss.str());
}
