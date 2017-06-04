#ifndef CALIBRATION_H
#define CALIBRATION_H

#define DEPTH_RESOLUTION_X 640
#define DEPTH_RESOLUTION_Y 480

#define COLOR_RESOLUTION_X 1920
#define COLOR_RESOLUTION_Y 1080

#include <string>

#include <OpenNI.h>
#include <NiTE.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "coordinates_mapper.h"

int findFloor(openni::Device* p_to_device, openni::VideoStream* p_to_vs_color, openni::VideoStream* p_to_vs_depth, CoordinatesMapper* p_to_coordmapper, nite::UserTracker* p_to_user_tracker, const double rgb_scaling, cv::Point3d& floor_origin, cv::Point3d& floor_normal);

int calibrate(const cv::Mat& m_pattern, openni::Device *p_to_device, openni::VideoStream *p_to_vs_color, const cv::Size& corners, const double scale_factor, cv::Mat& m_calibration);

#endif //CALIBRATION_H
