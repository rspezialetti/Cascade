#include "..\include\coordinates_mapper.h"

#include <boost\lexical_cast.hpp>

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

void CoordinatesMapper::projectCoordinatesToProjector(const cv::Point3d& point_in_millimiters, cv::Point2i& point_in_pixels)
{
	std::vector<cv::Point2d> v_p_out;
	if(use_calibration_)
	{
		//Point on the dance floor like blood
		cv::Point3d point_on_floor;
		pointToPlaneProjection(floor_, floor_normal_, point_in_millimiters, point_on_floor);

		//Point in RGB
		cv::Point2i point_in_rgb;
		projectRealWordlToRgb(point_on_floor, point_in_rgb, rgb_scale_);

		//Warping
		std::vector<cv::Point2d> v_p_in{ point_in_rgb };
		cv::perspectiveTransform(v_p_in, v_p_out, calibration_);

	}
	else
	{
		v_p_out.resize(1, cv::Point2d(point_in_millimiters.z / (double)max_range_z_, ((-point_in_millimiters.x + (max_range_x_*0.5)) / (double)max_range_x_) ));
	}

	const int point_x_int = static_cast<int>(std::floor(v_p_out[0].x));
	const int point_y_int = static_cast<int>(std::floor(v_p_out[0].y));

	point_in_pixels.x = std::min(point_x_int, projector_resolution_x_ - 1);
	//point_in_pixels.y = std::min(projector_resolution_y_ - point_y_int, projector_resolution_y_ - 1);
	point_in_pixels.y = std::min(point_y_int, projector_resolution_y_ - 1);

}