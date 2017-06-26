#include "calibration.h"

#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

int findFloor(openni::Device* p_to_device, openni::VideoStream* p_to_vs_color, openni::VideoStream* p_to_vs_depth, CoordinatesMapper* p_to_coordmapper, nite::UserTracker* p_to_user_tracker, const double rgb_scaling, cv::Point3d& floor_point, cv::Point3d& floor_normal) 
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

		cv::Size rgb_scaled(std::floor(COLOR_RESOLUTION_X / 1), std::floor(COLOR_RESOLUTION_Y / 1));

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

					//Copy plane normal
					floor_normal.x = frame_user_tracker.getFloor().normal.x;
					floor_normal.y = frame_user_tracker.getFloor().normal.y;
					floor_normal.z = frame_user_tracker.getFloor().normal.z;

					//Copy plane origin
					floor_point.x = frame_user_tracker.getFloor().point.x;
					floor_point.y = frame_user_tracker.getFloor().point.y;
					floor_point.z = frame_user_tracker.getFloor().point.z;
					
					float floor_depth_x = 0, floor_depth_y = 0; 
					p_to_user_tracker->convertJointCoordinatesToDepth(floor_point.x, floor_point.y, floor_point.z, &floor_depth_x, &floor_depth_y);
					
					float floor_rgb_x = 0, floor_rgb_y = 0;
					p_to_coordmapper->depthToColor(floor_depth_x, floor_depth_y, floor_point.z, floor_rgb_x, floor_rgb_y);
					
					cv::Point2i floor_rgb;
					floor_rgb.x = static_cast<int>(std::max(0.f, std::min(1.f *COLOR_RESOLUTION_X-1, floor_rgb_x)));
					floor_rgb.y = static_cast<int>(std::max(0.f, std::min(1.f *COLOR_RESOLUTION_Y-1, floor_rgb_y)));

					const nite::Array<nite::UserData>& users = frame_user_tracker.getUsers();

					cv::Point3d usr_com, usr_com_on_floor;
					bool user_found = false;
					for (int i = 0; i < users.getSize(); ++i)
					{
						const nite::UserData& user = users[i];

						if (user.isVisible() && !user.isLost())
						{
							user_found = true;

							usr_com.x = user.getCenterOfMass().x;
							usr_com.y = user.getCenterOfMass().y;
							usr_com.z = user.getCenterOfMass().z;
						}
						break;
					}

					if (m_rgb_resized.data)
					{
						if (floor_rgb.x != 0 && floor_rgb.y != 0)
							cv::circle(m_rgb_resized, floor_rgb, 10, cv::Scalar(0, 255, 0), CV_FILLED, 8);
					
						if (user_found) 
						{
							p_to_coordmapper->setFloor(floor_point);
							p_to_coordmapper->setFloorNormal(floor_normal);
	
							cv::Point2i usr_com_rgb;
							p_to_coordmapper->projectUserToProjector(p_to_user_tracker, usr_com, false, usr_com_rgb);
							cv::circle(m_rgb_resized, usr_com_rgb, 10, cv::Scalar(255, 255, 0), CV_FILLED, 8);
							
							//Draw user on depth
							float usr_vis_x = 0, usr_vis_y = 0;
							p_to_user_tracker->convertJointCoordinatesToDepth(usr_com.x, usr_com.y, usr_com.z, &usr_vis_x, &usr_vis_y);
							cv::Point2i com_to_vis(static_cast<int>(usr_vis_x), static_cast<int>(usr_vis_y));

							cv::circle(m_depth_colored, com_to_vis, 10, cv::Scalar(255, 0, 0), CV_FILLED, 8);
						}
						//Show RGB at half size
						cv::resizeWindow("Rgb", COLOR_RESOLUTION_X*0.5, COLOR_RESOLUTION_Y*0.5);
						
						cv::Mat rgb_for_vis;
						cv::resize(m_rgb_resized, rgb_for_vis, cv::Size(COLOR_RESOLUTION_X*0.5, COLOR_RESOLUTION_Y*0.5));
						cv::imshow("Rgb", rgb_for_vis);
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

	if (findChessboardCorners(m_pattern, corners, v_corners_pattern, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE))
	{
		drawChessboardCorners(m_pattern, corners, cv::Mat(v_corners_pattern), true);

		openni::VideoStream** stream = new openni::VideoStream*[1];
		stream[0] = p_to_vs_color;
	
		puts("Kinect initialization completed.");
		
		cv::Mat m_rgb(cv::Size(COLOR_RESOLUTION_X, COLOR_RESOLUTION_Y), CV_8UC3);
		cv::Mat m_rgb_resized;
	
		openni::VideoFrameRef frame_color;

		cv::namedWindow("Rgb", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("Pattern", CV_WINDOW_NORMAL);
		cv::resizeWindow("Pattern", m_pattern.cols/ 3, m_pattern.rows / 3);
	
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
					//cv::flip(m_rgb, m_rgb, 1);

					if (m_rgb.data)
					{
						cv::cvtColor(m_rgb, m_rgb, CV_BGR2RGB);

						cv::resize(m_rgb, m_rgb_resized, rgb_scaled);

						if (findChessboardCorners(m_rgb_resized, corners, v_corners_acquisition))
						{
							cv::Mat m_rgb_gray;
							cvtColor(m_rgb_resized, m_rgb_gray, CV_BGR2GRAY);
							cv::cornerSubPix(m_rgb_gray, v_corners_acquisition, corners, cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
							   
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

		//cv::Mat m_pattern_dist;
		//cv::warpPerspective(m_pattern, m_pattern_dist, m_calibration, m_pattern.size());

		std::vector<cv::Point2f> v_corners_warped;
		cv::Mat inv = m_calibration;
		cv::perspectiveTransform(v_corners_acquisition, v_corners_warped, inv.inv());
		std::ofstream corners("corners.txt");
		for (size_t i_c = 0; i_c <v_corners_warped.size(); ++i_c)
		{
			cv::circle(m_pattern, v_corners_warped[i_c], 10, cv::Scalar(255, 255, 0), CV_FILLED, 1);
			corners << v_corners_warped[i_c] << std::endl;
		}
		corners.close();

		cv::namedWindow("Pattern warped", cv::WINDOW_NORMAL);
		cv::imshow("Pattern warped", m_pattern);
		cv::resizeWindow("Pattern warped", m_pattern.cols / 3, m_pattern.rows / 3);

		cv::waitKey(0);

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
