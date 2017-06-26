#include "coordinates_mapper.h"

#include <boost\lexical_cast.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>

CoordinatesMapper::CoordinatesMapper(std::map<std::string, std::string>& values)
{
		fx_ = boost::lexical_cast<double>(values["fx"]);
		fy_ = boost::lexical_cast<double>(values["fy"]);
		cx_ = boost::lexical_cast<double>(values["cx"]);
		cy_ = boost::lexical_cast<double>(values["cy"]);

		projector_resolution_x_ = boost::lexical_cast<int>(values["resolution_projector_x"]);
		projector_resolution_y_ = boost::lexical_cast<int>(values["resolution_projector_y"]);

		rgb_scale_ = boost::lexical_cast<double>(values["rgb_scale_factor"]);

		rgb_resolution_x_ = boost::lexical_cast<int>(values["rgb_resolution_x"]);
		rgb_resolution_y_ = boost::lexical_cast<int>(values["rgb_resolution_y"]);

}

bool CoordinatesMapper::init()
{
	bool is_ready = true;

	std::string error_message = "CoordinatesMapper init errors: ";

	if (use_calibration_)
	{
		if (calibration_.data == 0)
		{
			error_message += " calibration matrix is empty.";
			is_ready = false;
		}

		if (fx_ == 0)
		{
			error_message += " camera intrinsic FX is zero.";
			is_ready = false;
		}

		if (fy_ == 0)
		{
			error_message += " camera intrinsic FY is zero.";
			is_ready = false;
		}

		if (cx_ == 0)
		{
			error_message += " camera intrinsic CX is zero.";
			is_ready = false;
		}

		if (cy_ == 0)
		{
			error_message += " camera intrinsic CY is zero.";
			is_ready = false;
		}

		if (rgb_scale_ == 0)
		{
			error_message += " RGB scale is zero.";
			is_ready = false;
		}
	
	}
	else
	{
		if (max_range_z_ == 0)
		{
			error_message += " camera range on z is zero.";
			is_ready = false;
		}

		if (max_range_x_ == 0)
		{
			error_message += " camera range on x is zero.";
			is_ready = false;
		}
	}

	if (!is_ready)
		std::cout << error_message << std::endl;

	return is_ready;
}

void CoordinatesMapper::projectUserToProjector(const cv::Point3d& point_in_millimiters, cv::Point2i& point_in_pixels)
{
		
	const int y = static_cast<int>((projector_resolution_y_ - 1) - (projector_resolution_y_ - 1) *((point_in_millimiters.z / (double)max_range_z_)));
	const int x = static_cast<int>((projector_resolution_x_ - 1) - ((projector_resolution_x_ - 1) *((-point_in_millimiters.x + (max_range_x_*0.5)) / (double)max_range_x_)));

	point_in_pixels.x = std::max<int>(0, std::min<int>(x, projector_resolution_x_ - 1));
	point_in_pixels.y = std::max<int>(0, std::min<int>(y, projector_resolution_y_ - 1));
}

//void CoordinatesMapper::projectCoordinatesToProjector(const cv::Point2d& point_in_rgb, cv::Point2i& point_in_pixels)
//{
//	std::vector<cv::Point2d> v_p_out;
//
//	//Warping
//	std::vector<cv::Point2d> v_p_in{ cv::Point2d(point_in_rgb.x/rgb_scale_, point_in_rgb.y/rgb_scale_) };
//	
//	cv::perspectiveTransform(v_p_in, v_p_out, calibration_);
//
//	const int point_x_int = static_cast<int>(std::floor(v_p_out[0].x));
//	const int point_y_int = static_cast<int>(std::floor(v_p_out[0].y));
//
//	point_in_pixels.x = std::max(0, std::min(point_x_int, projector_resolution_x_ - 1));
//	//point_in_pixels.y = std::max(0, std::min(projector_resolution_y_ - point_y_int, projector_resolution_y_ - 1));
//	point_in_pixels.y = std::max(0, std::min(point_y_int, projector_resolution_y_ - 1));
//}

void CoordinatesMapper::projectUserToProjector(const nite::UserTracker *p_to_usr_tracker, const cv::Point3d& point_in_mm, const bool warp, cv::Point2i& point_in_pixels)
{
	cv::Point3d point_mm_floor;
	pointToPlaneProjection(floor_, floor_normal_, point_in_mm, point_mm_floor);

	float usr_depth_x = 0, usr_depth_y = 0;
	p_to_usr_tracker->convertJointCoordinatesToDepth(point_mm_floor.x, point_mm_floor.y, point_mm_floor.z, &usr_depth_x, &usr_depth_y);

	float usr_rgb_x = 0;
	float usr_rgb_y = 0;
	depthToColor(usr_depth_x, usr_depth_y, point_mm_floor.z, usr_rgb_x, usr_rgb_y);

	//Add warping
	if (warp) 
	{
		//Warping
		std::vector<cv::Point2d> v_p_in{ cv::Point2d(usr_rgb_x /(float)rgb_scale_, usr_rgb_y /(float)rgb_scale_) };
		std::vector<cv::Point2d> v_p_out;
		cv::perspectiveTransform(v_p_in, v_p_out, calibration_);
		
		usr_rgb_x = v_p_out[0].x;
		usr_rgb_y = (std::floor((projector_resolution_y_ -1) *0.5)) - v_p_out[0].y;
	}
	
	point_in_pixels.x = static_cast<int>(std::max<int>(0, std::min<int>(std::floor(usr_rgb_x), COLOR_RESOLUTION_X - 1)));
	point_in_pixels.y = static_cast<int>(std::max<int>(0, std::min<int>(std::floor(usr_rgb_y), COLOR_RESOLUTION_Y - 1)));

}