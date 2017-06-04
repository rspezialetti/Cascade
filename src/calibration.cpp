#include "calibration.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

int findFloor(openni::Device* p_to_device, openni::VideoStream* p_to_vs_color, openni::VideoStream* p_to_vs_depth, CoordinatesMapper* p_to_coordmapper, nite::UserTracker* p_to_user_tracker, const double rgb_scaling, cv::Point3d& floor_origin, cv::Point3d& floor_normal) 
{
		openni::VideoStream** stream = new openni::VideoStream*[2];
		stream[0] = p_to_vs_color;
		stream[1] = p_to_vs_depth;

		puts("Kinect initialization completed.");

		cv::Mat m_rgb(cv::Size(COLOR_RESOLUTION_X, COLOR_RESOLUTION_Y), CV_8UC3);
		cv::Mat m_rgb_resized;

		cv::Mat m_depth(cv::Size(DEPTH_RESOLUTION_X, DEPTH_RESOLUTION_Y), CV_16UC1);
		cv::Mat m_depth_colored;

		openni::VideoFrameRef frame_color;
		openni::VideoFrameRef frame_depth;

		cv::namedWindow("Rgb", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE);

		int index_changed;

		cv::Size rgb_scaled(std::floor(COLOR_RESOLUTION_X / rgb_scaling), std::floor(COLOR_RESOLUTION_Y / rgb_scaling));

		while (p_to_device->isValid())
		{
			openni::OpenNI::waitForAnyStream(stream, 2, &index_changed);

			switch (index_changed)
			{
			case 0:
			{
				p_to_vs_color->readFrame(&frame_color);

				if (frame_color.isValid())
				{
					m_rgb.data = (uchar*)frame_color.getData();

					if (m_rgb.data)
					{
						cv::cvtColor(m_rgb, m_rgb, CV_BGR2RGB);
						cv::resize(m_rgb, m_rgb_resized, rgb_scaled);
					}
					else
					{
						return -1;
					}
				}
			}
			break;

			case 1:
			{
				p_to_vs_depth->readFrame(&frame_depth);

				if (frame_depth.isValid())
				{
					m_depth.data = (uchar*)frame_depth.getData();

					double min, max;
					cv::minMaxIdx(m_depth, &min, &max);

					m_depth.convertTo(m_depth_colored, CV_8U, 255 / (max - min), -min);
					cv::cvtColor(m_depth_colored, m_depth_colored, cv::COLOR_GRAY2RGB);

					nite::UserTrackerFrameRef frame_user_tracker;
					p_to_user_tracker->readFrame(&frame_user_tracker);

					const nite::Plane floor = frame_user_tracker.getFloor();

					//Copy plane normal
					floor_normal.x = floor.normal.x;
					floor_normal.y = floor.normal.y;
					floor_normal.z = floor.normal.z;

					//Copy plane origin
					floor_origin.x = floor.point.x;
					floor_origin.y = floor.point.y;
					floor_origin.z = floor.point.z;

					cv::Point2i floor_rgb, user_rgb(-1,-1);
					p_to_coordmapper->projectRealWordlToRgb(floor_origin, floor_rgb, rgb_scaling);

					const nite::Array<nite::UserData>& users = frame_user_tracker.getUsers();

					cv::Point3d user_com;
					for (int i = 0; i < users.getSize(); ++i)
					{
						const nite::UserData& user = users[i];

						if (user.isVisible() && !user.isLost())
						{
							user_com.x = user.getCenterOfMass().x;
							user_com.y = user.getCenterOfMass().y;
							user_com.z = user.getCenterOfMass().z;
						}
						break;
					}

					p_to_coordmapper->projectRealWordlToRgb(user_com, user_rgb, rgb_scaling);

					if (m_rgb_resized.data)
					{
						cv::circle(m_rgb_resized, floor_rgb, 10, cv::Scalar(0, 255, 0), CV_FILLED, 8);

						if(user_rgb.x != -1 && user_rgb.y != -1)
							cv::circle(m_rgb_resized, user_rgb, 10, cv::Scalar(255, 0, 0), CV_FILLED, 8);

						cv::imshow("Rgb", m_rgb_resized);
					}
				
					cv::imshow("Depth", m_depth_colored);			
				}
				else 
				{
					return -1;
				}
			}
			break;

			default:
				puts("Error retrieving a stream");
			}
			if ((char)cv::waitKey(1) == 99)
				break;
		}
	
		cv::destroyWindow("Rgb");
		cv::destroyWindow("Depth");

	return 1;
}

int calibrate(const cv::Mat& m_pattern, openni::Device *p_to_device, openni::VideoStream *p_to_vs_color, const cv::Size& corners, const double scale_factor, cv::Mat& m_calibration)
{

	std::vector<cv::Point2f> v_corners_pattern;
	std::vector<cv::Point2f> v_corners_acquisition;

	if (findChessboardCorners(m_pattern, corners, v_corners_pattern))
	{
		drawChessboardCorners(m_pattern, corners, cv::Mat(v_corners_pattern), true);

		openni::VideoStream** stream = new openni::VideoStream*[1];
		stream[0] = p_to_vs_color;
	
		puts("Kinect initialization completed.");
		
		cv::Mat m_rgb(cv::Size(COLOR_RESOLUTION_X, COLOR_RESOLUTION_Y), CV_8UC3);
		cv::Mat m_rgb_resized;
	
		openni::VideoFrameRef frame_color;

		cv::namedWindow("Rgb", CV_WINDOW_AUTOSIZE);
	
		int index_changed;

		bool calibrate = false;

		cv::Size rgb_scaled(std::floor(COLOR_RESOLUTION_X / scale_factor), std::floor(COLOR_RESOLUTION_Y / scale_factor));

		while (p_to_device->isValid() && !calibrate)
		{
			openni::OpenNI::waitForAnyStream(stream, 1, &index_changed);

			switch (index_changed)
			{
			case 0:
			{
				p_to_vs_color->readFrame(&frame_color);

				if (frame_color.isValid())
				{
					m_rgb.data = (uchar*)frame_color.getData();
				
					if (m_rgb.data)
					{
						cv::cvtColor(m_rgb, m_rgb, CV_BGR2RGB);

						cv::resize(m_rgb, m_rgb_resized, rgb_scaled);

						if (findChessboardCorners(m_rgb_resized, corners, v_corners_acquisition))
						{
							drawChessboardCorners(m_rgb_resized, corners, cv::Mat(v_corners_acquisition), true);
							if ((char)cv::waitKey(1) == 99)
							{
								calibrate = true;
								break;
							}
						}
						else
						{
							std::cout << "Corners not found" << std::endl;
						}
						
						cv::imshow("Pattern", m_pattern);
						cv::imshow("Rgb", m_rgb_resized);
					}
					else
					{
						return -3;
					}
				}
			}
			break;
			

			default:
				puts("Error retrieving a stream");
			}

			cv::waitKey(40);
		}
		m_calibration = cv::findHomography(v_corners_pattern, v_corners_acquisition, 0);

		cv::Mat m_pattern_dist;
		cv::warpPerspective(m_pattern, m_pattern_dist, m_calibration, m_pattern.size());

		cv::namedWindow("Pattern warped", cv::WINDOW_AUTOSIZE);
		cv::imshow("Pattern warped", m_pattern_dist);

		cv::waitKey(1);

		cv::destroyWindow("Rgb");
		cv::destroyWindow("Pattern");
		cv::destroyWindow("Pattern warped");
	}
	else
	{
		std::cout << "Corners on pattern not found" << std::endl;
		return -1;
	}

	return 1;
}
