#ifndef COORDINATES_MAPPER_H
#define COORDINATES_MAPPER_H

#define DEPTH_RESOLUTION_X 640
#define DEPTH_RESOLUTION_Y 480

#define COLOR_RESOLUTION_X 1920
#define COLOR_RESOLUTION_Y 1080

#include <vector>
#include <map>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <NiTE.h>

static const float depth_q = 0.01;
static const float color_q = 0.002199;

class CoordinatesMapper
{
public:
	CoordinatesMapper() : use_calibration_(true), floor_(0.0, 0.0, 0.0), fx_(0.0), fy_(0.0), cx_(0.0), cy_(0.0), max_range_z_(0.0), max_range_x_(0.0), projector_resolution_x_(1920), projector_resolution_y_(1080), rgb_scale_(0), rgb_resolution_x_(1920), rgb_resolution_y_(1080)
	{};

	CoordinatesMapper(std::map <std::string, std::string>& values);

	~CoordinatesMapper() {};

	void projectUserToProjector(const cv::Point3d& point_in_millimiters, cv::Point2i& point_in_pixels);

	void projectUserToProjector(const nite::UserTracker *p_to_usr_tracker, const cv::Point3d& point_in_mm, const bool warp, cv::Point2i& point_in_pixels);

	//void projectCoordinatesToProjector(const cv::Point2d& point_in_rgb, cv::Point2i& point_in_pixels);

	inline void setUseCalibration(bool use_calibration) { use_calibration_ = use_calibration; }

	inline void setCalibrationMatrix(cv::Mat& calibration) { calibration_ = calibration; }

	inline void setFloor(cv::Point3d& floor) { floor_ = floor; }

	inline void setFloorNormal(cv::Point3d& floor_normal) { floor_normal_ = floor_normal; }

	/*In order fx, fy, cx, cy*/
	inline void setRGBIntrinsic(std::vector<double> v_camera_intrinsic)
	{
		fx_ = v_camera_intrinsic[0];
		fy_ = v_camera_intrinsic[1];
		cx_ = v_camera_intrinsic[2];
		cy_ = v_camera_intrinsic[3];
	}

	inline void setMaxRangeX(double max_range_x) { max_range_x_ = max_range_x; }

	inline void setMaxRangeZ(double max_range_z) { max_range_z_ = max_range_z; }

	inline void setProjectoResolutionX(int projector_resolution_x) { projector_resolution_x_ = projector_resolution_x; }

	inline void setProjectoResolutionY(int projector_resolution_y) { projector_resolution_y_ = projector_resolution_y; }

	inline void setRGBScale(double rgb_scale) { rgb_scale_ = rgb_scale; }

	inline void setIntrinsic(cv::Mat& intrinsic) { intrinsic_ = intrinsic; }

	inline void setDistCoeffs(cv::Mat& dist_coeffs) { dist_coeffs_ = dist_coeffs; }

	inline void setRVecs(std::vector<cv::Mat>& rvec) { rvecs_ = rvec; }

	inline void setTVecs(std::vector<cv::Mat>& tvec) { tvecs_ = tvec; }

	inline cv::Mat getIntrinsic() const { return intrinsic_; }

	inline cv::Mat getDistCoeffs () const { return dist_coeffs_; }

	//inline void realWorldToDepth(const cv::Point3d& point_mm, cv::Point2i& point_depth)
	//{
	//	const float cx_depth = 257.322; const float cy_depth = 204.137;
	//	const float fx_depth = 364.764; const float fy_depth = 364.764;
	//	
	//	const float k1_depth = 0.09343, k2_depth = -0.272194, k3_depth = 0.0925756;
	//	const float p1_depth = 0.f, p2_depth = 0.f;

	//	const float x = (point_mm.x * fx_depth / point_mm.z) + cx_depth;
	//	const float y = (-point_mm.y * fy_depth / point_mm.z) + cy_depth;

	//	float dx = ((float)x - cx_depth) / fx_depth;
	//	float dy = ((float)y - cy_depth) / fy_depth;
	//	float dx2 = dx * dx;
	//	float dy2 = dy * dy;
	//	float r2 = dx2 + dy2;
	//	float dxdy2 = 2 * dx * dy;
	//	float kr = 1 + ((k3_depth * r2 + k2_depth) * r2 + k1_depth) * r2;
	//	
	//	const float x_depth =  fx_depth * (dx * kr + p2_depth * (r2 + 2 * dx2) + p1_depth * dxdy2) + cx_depth;
	//	const float y_depth = fy_depth * (dy * kr + p1_depth * (r2 + 2 * dy2) + p2_depth * dxdy2) + cy_depth;

	//	point_depth.x = static_cast<int>(std::max(0.f, std::min(x_depth, 1.0f * 640 - 1)));
	//	point_depth.y = static_cast<int>(std::max(0.f, std::min(y_depth, 1.0f * 480 - 1)));
	//}

	inline void depthToColor(float mx, float my, float z, float& rx, float& ry)
	{
		if (z == 0)
			return;
			 

		const float shift_d = 863.0f;
		const float shift_m = 52.0f;

		const float mx_x3y0 = 0.000505688;
		const float mx_x0y3 = 3.46255e-05;
		const float mx_x2y1 = 5.74793e-05;
		const float mx_x1y2 = 0.000382165;
		const float mx_x2y0 = 0.00106235;
		const float mx_x0y2 = -4.27285e-06;
		const float mx_x1y1 = -0.000254386;
		const float mx_x1y0 = 0.639775;
		const float mx_x0y1 = -0.00200164;
		const float mx_x0y0 = 0.158617;
		const float my_x3y0 = 1.99254e-05;
		const float my_x0y3 = 0.000839958;
		const float my_x2y1 = 0.000453947;
		const float my_x1y2 = 6.26741e-05;
		const float my_x2y0 = -0.000207281;
		const float my_x0y2 = -0.000462894;
		const float my_x1y1 = 0.00106286;
		const float my_x1y0 = 0.00160405;
		const float my_x0y1 = 0.639276;
		const float my_x0y0 = 0.0217857;

		const float color_cx = 959.5;
		const float	color_cy = 539.5;
		const float color_fx = 1081.37;

		const float depth_cx = 257.322;
		const float depth_cy = 204.137;

		mx = (mx - depth_cx) * depth_q;
		my = (my - depth_cy) * depth_q;

		float wx =
			(mx * mx * mx * mx_x3y0) + (my * my * my * mx_x0y3) +
			(mx * mx * my * mx_x2y1) + (my * my * mx * mx_x1y2) +
			(mx * mx * mx_x2y0) + (my * my * mx_x0y2) + (mx * my * mx_x1y1) +
			(mx * mx_x1y0) + (my * mx_x0y1) + (mx_x0y0);

		float wy =
			(mx * mx * mx * my_x3y0) + (my * my * my * my_x0y3) +
			(mx * mx * my * my_x2y1) + (my * my * mx * my_x1y2) +
			(mx * mx * my_x2y0) + (my * my * my_x0y2) + (mx * my * my_x1y1) +
			(mx * my_x1y0) + (my * my_x0y1) + (my_x0y0);

		rx = (wx / (color_fx * color_q)) - (shift_m / shift_d);

		rx += (shift_m / z);
		rx = rx * color_fx + color_cx;

		ry = (wy / color_q) + color_cy;
	}

	//inline void projectRealWordlToRgb(const cv::Point3d& point_to_project, cv::Point2i& point_projected, const double scale_rgb) const
	//{
	//	point_projected.x = 0;
	//	point_projected.y = 0;

	//	int resolution_scaled_x = (rgb_scale_ > 0) ? std::floor(rgb_resolution_x_ / (double)rgb_scale_) : rgb_resolution_x_;
	//	int resolution_scaled_y = (rgb_scale_ > 0) ? std::floor(rgb_resolution_y_ / (double)rgb_scale_) : rgb_resolution_y_;

	//	if (point_to_project.z >0 && !isnan<double>(point_to_project.z))
	//	{
	//		point_projected.x = (point_to_project.x * fx_ / point_to_project.z) + cx_;
	//		////kinect axis are different from opencv image 
	//		point_projected.y = (-point_to_project.y * fy_ / point_to_project.z) + cy_;
	//	
	//		std::vector<cv::Point2d> point_2d;
	//		std::vector<cv::Point3d> v_in{ point_to_project };
	//		//cv::projectPoints(v_in, rvecs_[0], tvecs_[0], intrinsic_, dist_coeffs_, point_2d);
	//		//point_projected.y = 1080 - std::floor(point_2d[0].y / scale_rgb);
	//		//point_projected.x = std::floor(point_2d[0].x / scale_rgb);

	//		point_projected.x = std::floor(point_projected.x / scale_rgb);
	//		point_projected.y = std::floor(point_projected.y / scale_rgb);

	//		point_projected.x = std::min<int>(resolution_scaled_x - 1, point_projected.x);
	//		point_projected.y = std::min<int>(resolution_scaled_y - 1, point_projected.y);

	//	}
	//}

	bool init();

	inline void pointToPlaneProjection(const cv::Point3d& plane_origin, const cv::Point3d& plane_normal, const cv::Point3d& point_to_project, cv::Point3d& point_projected)
	{
		cv::Point3d point_on_origin = point_to_project - plane_origin;

		const double dot = point_on_origin.dot(plane_normal);
		point_projected = point_to_project - dot*plane_normal;
	}

private:

	bool use_calibration_;

	cv::Mat calibration_;

	cv::Point3d floor_;

	cv::Point3d floor_normal_;

	double fx_;
	double fy_;
	double cx_;
	double cy_;

	double max_range_z_;
	
	double max_range_x_;

	double rgb_scale_;

	int projector_resolution_x_;
	int projector_resolution_y_;

	int rgb_resolution_x_;
	int rgb_resolution_y_;

	std::vector<cv::Mat> rvecs_, tvecs_;
	cv::Mat intrinsic_, dist_coeffs_;


};
#endif