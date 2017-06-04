#include <OpenNI.h>
#include <NiTE.h>

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/circular_buffer.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "calibration.h"
#include "coordinates_mapper.h"
#include "utils.h"
#include "UDPClient.hpp"

#define FRAMERATE 30
#define CIRCLE_SIZE 10

#define MAX_USERS 6
#define MAX_FRAME 90

const int colors[] = { 255, 0, 0, 0, 255, 0, 128, 0, 0, 255, 255, 255, 255, 0, 255, 0, 0, 255};

std::vector<float> trackUsers(nite::UserTracker *p_user_tracker, openni::Device* p_to_device, const openni::VideoStream *p_video_stream_depth, openni::VideoFrameRef& frame_depth)
{
	std::vector<float> v_users_coord_in_mm;

	nite::UserTrackerFrameRef frame_user_tracker;
	nite::Status tracker_status;

	//Read frame from user tracker
	tracker_status = p_user_tracker->readFrame(&frame_user_tracker);

	if (tracker_status != nite::STATUS_OK)
	{
		printf("GetNextData failed\n");
	}

	frame_depth = frame_user_tracker.getDepthFrame();

	if (frame_depth.isValid())
	{
		/*Track user*/
		const nite::Array<nite::UserData>& users = frame_user_tracker.getUsers();

		for (int i = 0; i < users.getSize(); ++i)
		{
			const nite::UserData& user = users[i];
			
			if (user.isVisible() && !user.isLost())
			{
				v_users_coord_in_mm.push_back(user.getCenterOfMass().x);
				v_users_coord_in_mm.push_back(user.getCenterOfMass().y);
				v_users_coord_in_mm.push_back(user.getCenterOfMass().z);
			}
		}
	}

	return v_users_coord_in_mm;
}

std::vector<float> getStillUsers(nite::UserTracker *p_user_tracker, openni::Device* p_to_device, const openni::VideoStream *p_video_stream_depth, openni::VideoFrameRef& frame_depth, boost::circular_buffer<std::vector<std::vector<float>>>& buffer,const double threshold_stdv)
{
	std::vector<float> v_users_coord_in_mm;

	nite::UserTrackerFrameRef frame_user_tracker;
	nite::Status tracker_status;

	//Read frame from user tracker
	tracker_status = p_user_tracker->readFrame(&frame_user_tracker);

	if (tracker_status != nite::STATUS_OK)
	{
		printf("GetNextData failed\n");
	}

	frame_depth = frame_user_tracker.getDepthFrame();
	int max_users = 0;

	std::vector<std::vector<float>> v_users_com(MAX_USERS, std::vector<float>{0.f, 0.f, 0.f});
	std::vector<int> v_id_users;

	if (frame_depth.isValid())
	{
		/*Track user*/
		const nite::Array<nite::UserData>& users = frame_user_tracker.getUsers();

		max_users = users.getSize();
		v_id_users.resize(max_users, 0);

		for (int i = 0; i < users.getSize(); ++i)
		{
			const nite::UserData& user = users[i];

			const int id_user = user.getId();
			v_id_users[i] = id_user;

			v_users_com[id_user] = std::vector<float>{ user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z };
		}

		buffer.push_back(v_users_com);
	}

	//Now analysis on users
	for (int i_u = 0; i_u < max_users; ++i_u)
	{
		std::vector<double> v_last_user_coord(3 * buffer.size(), 0.0);
		for (size_t i_b = 0; i_b < buffer.size(); ++i_b)
		{
			v_last_user_coord[0 + (i_b * 3)] = buffer[i_b][i_u][0];
			v_last_user_coord[1 + (i_b * 3)] = buffer[i_b][i_u][1];
			v_last_user_coord[2 + (i_b * 3)] = buffer[i_b][i_u][2];
		}

		double std = calculateSTDV<double>(v_last_user_coord);

		if (std > 0.0 && std < threshold_stdv)
		{
			const int id_user = v_id_users[i_u];

			v_users_coord_in_mm.push_back(v_users_com[id_user][0]);
			v_users_coord_in_mm.push_back(v_users_com[id_user][1]);
			v_users_coord_in_mm.push_back(v_users_com[id_user][2]);
		}

	}

	return v_users_coord_in_mm;
}


/*Create correct video stream for calibration or user tracking*/
openni::Status createVideoStream(const int mode, const std::string kinect_id, openni::Device* p_to_device, openni::VideoStream *p_to_video_stream)
{
	//Acquire from depth sensor for calibration
	openni::VideoMode params_videomode;
	params_videomode.setFps(FRAMERATE);
	openni::Status status_creation;

	if (mode) 
	{
		status_creation = p_to_video_stream->create(*p_to_device, openni::SENSOR_DEPTH);
		puts("Depth sensor start.");

		//Video mode for depth		
		//paramvideo.setResolution(512, 424);	
		params_videomode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_100_UM);
	}
	else
	{
		/*read data from kinect*/
		status_creation = p_to_video_stream->create(*p_to_device, openni::SENSOR_COLOR);
		puts("RGB sensor start.");

		//Video mode for color
		params_videomode.setResolution(COLOR_RESOLUTION_X, COLOR_RESOLUTION_Y);
		params_videomode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	}

	p_to_video_stream->setVideoMode(params_videomode);
	p_to_device->setDepthColorSyncEnabled(false);
	
	return status_creation;
}


void shutDown(CoordinatesMapper* p_to_coord_mapper, openni::VideoStream* p_to_depth_stream, openni::Device* p_to_device)
{
	delete p_to_coord_mapper;

	//Stop and destroy video stream
	p_to_depth_stream->stop();
	p_to_depth_stream->destroy();
	delete p_to_depth_stream;

	//Close device
	p_to_device->close();
	delete p_to_device;
}

int main(int argc, char** argv)
{
	std::cout << "Start"<< std::endl;

	//MODE 0 calibration and 1 user tracking
	int mode = -1;
	std::string kinect_id = "";

	/*Read command line*/
	if (!readCmdLine(argc, argv, mode, kinect_id))
		return -1;
	
	/*Read Configuration File*/
	const std::string path_to_conf_file = "../data/config.txt";
	std::map<std::string, std::string> m_init_values;

	if (!readConfigurationFile(path_to_conf_file, m_init_values))
	{
		std::cerr << "Impossible to read: "<< path_to_conf_file << std::endl;
		return -1;
	}
			
	/*Set parameters*/
	CoordinatesMapper *p_to_coord_mapper = new CoordinatesMapper(m_init_values);

	if (!boost::lexical_cast<int>(m_init_values["use_calibration"]))
	{
		p_to_coord_mapper->setMaxRangeX(boost::lexical_cast<int>(m_init_values["max_x_range"]));
		p_to_coord_mapper->setMaxRangeZ(boost::lexical_cast<int>(m_init_values["max_z_range"]));
	}

	/*Openni Init*/
	openni::Status status_openni = openni::OpenNI::initialize();
	if (status_openni != openni::STATUS_OK)
	{
		printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
		return status_openni;
	}

	openni::Device* p_to_device = new openni::Device;

	if (kinect_id != "")
	{
		openni::Array<openni::DeviceInfo> devices_info;
		openni::OpenNI::enumerateDevices(&devices_info);

		for (int i_d = 0; i_d < devices_info.getSize(); ++i_d)
		{
			const std::string uri = devices_info[i_d].getUri();
			if (uri.find(kinect_id) != std::string::npos)
			{
				status_openni = p_to_device->open(devices_info[i_d].getUri());
				break;
			}
		}
	}
	else
	{    
		status_openni = p_to_device->open(openni::ANY_DEVICE);
	}

	if (status_openni != openni::STATUS_OK)
	{
		printf("Failed to open device\n %s \n", openni::OpenNI::getExtendedError());
		return status_openni;
	}

	openni::VideoStream *p_to_depth_stream = new openni::VideoStream;
	status_openni = createVideoStream(1, kinect_id, p_to_device, p_to_depth_stream);

	if (status_openni != openni::STATUS_OK)
	{
		printf("Failed to create depth video stream \n %s \n", openni::OpenNI::getExtendedError());

		shutDown(p_to_coord_mapper, p_to_depth_stream, p_to_device);

		//Close OpeNI
		openni::OpenNI::shutdown();

		return status_openni;
	}

	//Start stream
	status_openni = p_to_depth_stream->start();
	if ( status_openni != openni::Status::STATUS_OK)
	{
		printf("Failed to start video stream\n %s \n", openni::OpenNI::getExtendedError());
		return status_openni;
	}

	nite::NiTE::initialize();
	nite::UserTracker *p_to_user_tracker = new nite::UserTracker();
	if (p_to_user_tracker->create(p_to_device) != nite::STATUS_OK)
	{
		printf("Failed to open user tracker\n");
		return openni::STATUS_ERROR;
	}

	/*Switch on operative mode*/
	cv::FileStorage file_storage;
	std::string error_message;
	cv::Mat calibration;

	//Take some parameters
	const std::string fn_calibration_matrix = m_init_values["file_calibration_matrix"];
	const bool visualize = boost::lexical_cast<int>(m_init_values["visualize_tracking"]);

	switch (mode) 
	{
		//Calibration
		case(0):
		{
			const std::string fn_pattern = m_init_values["file_calibration_pattern"];
			cv::Mat pattern = cv::imread(fn_pattern);
			cv::Point3d floor_origin, floor_normal;

			openni::VideoStream *p_to_color_stream = new openni::VideoStream;
			status_openni = createVideoStream(0, kinect_id, p_to_device, p_to_color_stream);

			if (status_openni != openni::STATUS_OK)
			{
				printf("Failed to create depth video stream \n %s \n", openni::OpenNI::getExtendedError());

				shutDown(p_to_coord_mapper, p_to_depth_stream, p_to_device);

				//Close OpeNI
				openni::OpenNI::shutdown();

				return status_openni;
			}

			//Start stream
			status_openni = p_to_color_stream->start();
			if (status_openni != openni::Status::STATUS_OK)
			{
				printf("Failed to start video stream\n %s \n", openni::OpenNI::getExtendedError());

				shutDown(p_to_coord_mapper, p_to_depth_stream, p_to_device);

				//Close OpeNI
				openni::OpenNI::shutdown();

				return status_openni;
			}

			if (pattern.data)
			{
				const int corner_x = boost::lexical_cast<int>(m_init_values["corners_calibration_x"]);
				const int corner_y = boost::lexical_cast<int>(m_init_values["corners_calibration_y"]);

				const double rgb_scale_factor = boost::lexical_cast<int>(m_init_values["rgb_scale_factor"]);

				//Find homograpghy
				const int result_calibration = calibrate(pattern, p_to_device, p_to_color_stream, cv::Size(corner_x, corner_y), rgb_scale_factor, calibration);
			
				//Find plane
				const int find_floor = findFloor(p_to_device, p_to_color_stream, p_to_depth_stream, p_to_coord_mapper, p_to_user_tracker, rgb_scale_factor, floor_origin, floor_normal);
				
				if (result_calibration  && find_floor)
				{
					//save calibration;
					if (file_storage.open(fn_calibration_matrix, cv::FileStorage::WRITE))
					{
						file_storage << "calibration" << calibration;
						file_storage << "floor_origin" << floor_origin;
						file_storage << "floor_normal" << floor_normal;

						file_storage.release();

						std::cout << "Calibration data saved in:" << fn_calibration_matrix << std::endl;
					}
					else 
					{
						error_message += "failed to open and write " + fn_calibration_matrix;
						ERROR_CALIBRATION_MESSAGE(error_message.c_str());
					}
				}
				else
				{
					error_message += "can't find cornners " + std::to_string(corner_x) + " - " + std::to_string(corner_y);
					error_message = (result_calibration == -1) ? error_message + " on chess board."  : error_message + " on image.";
					ERROR_CALIBRATION_MESSAGE(error_message.c_str());
				}
			}
			else
			{
				error_message = " can't load pattern " + fn_pattern;
				ERROR_CALIBRATION_MESSAGE(error_message.c_str());
			}

			p_to_color_stream->stop();
			p_to_color_stream->destroy();

			delete p_to_color_stream;

		}
			break;
		case(1): 
		{
			boost::asio::io_service io_service;

			const std::string address = m_init_values["server_address"];
			const std::string port = m_init_values["server_port"];

			const std::string fn_timestamp = "C:" + m_init_values["file_timestamp"];

			const int use_calibration = boost::lexical_cast<int>(m_init_values["use_calibration"]);

			const double th_std = 50;		
			boost::circular_buffer<std::vector<std::vector<float>>> buffer(MAX_FRAME);

			clock_t last_timestamp = time(0);
			writeTimestamp(fn_timestamp, last_timestamp);

			UDPClient *p_udp_client;

			try 
			{
				p_udp_client = new UDPClient(io_service, address, port, MAX_USERS);
			}
			catch(std::exception& exception)
			{
				std::cerr << "Impossibile to create Udp client" << std::endl;
				return -1;
			}
	
			cv::Mat calibration_inv;
			cv::Point3d plane_origin, plane_normal;
			if (use_calibration)
			{
				if (file_storage.open(fn_calibration_matrix, cv::FileStorage::READ))
				{
					file_storage["calibration"] >> calibration;
					calibration_inv = calibration.inv();

					file_storage["floor_origin"] >> plane_origin;
					file_storage["floor_normal"] >> plane_normal;

					p_to_coord_mapper->setCalibrationMatrix(calibration_inv);
					p_to_coord_mapper->setFloor(plane_origin);
					p_to_coord_mapper->setFloorNormal(plane_normal);
				}
				else
				{
					error_message += "failed to open and read from " + fn_calibration_matrix;
					ERROR_CALIBRATION_MESSAGE(error_message.c_str());
				}
			}

			if (p_to_device->getSensorInfo(openni::SENSOR_DEPTH) != NULL && p_to_device->isValid())
			{
				while (!GetAsyncKeyState(VK_ESCAPE))
				{
					openni::VideoFrameRef frame_depth;
					std::vector<float> v_users_cord_mm = trackUsers(p_to_user_tracker, p_to_device, p_to_depth_stream, frame_depth);

					//std::vector<float> v_users_cord_mm = getStillUsers(p_to_user_tracker, p_to_device, p_to_depth_stream, frame_depth, buffer, th_std);
					
					std::vector<int> v_users_cord_on_projector;
					for (size_t i_u = 0; i_u < v_users_cord_mm.size(); i_u += 3)
					{
						cv::Point2i user_on_projector;
						p_to_coord_mapper->projectCoordinatesToProjector(cv::Point3d(v_users_cord_mm[i_u], v_users_cord_mm[i_u + 1], v_users_cord_mm[i_u + 2]), user_on_projector);

						v_users_cord_on_projector.push_back(user_on_projector.x);
						v_users_cord_on_projector.push_back(user_on_projector.y);
					}

					if (v_users_cord_on_projector.size() > 0)
					{
						//send
						try
						{
							p_udp_client->send(v_users_cord_on_projector);
						}
						catch (std::exception& e)
						{
							std::cerr << e.what() << std::endl;
						}
					}

					//save data for watchdog
					const clock_t current_timestamp = time(0);
					if (difftime(current_timestamp, last_timestamp) > 30)
					{
						last_timestamp = current_timestamp;
						
						//save timestamp
						writeTimestamp(fn_timestamp, last_timestamp);
					}

					//visualization
					if (visualize) 
					{
						cv::Mat m_depth(cv::Size(frame_depth.getWidth(), frame_depth.getHeight()), CV_16UC1);
						m_depth.data = (uchar*)frame_depth.getData();

						double min, max;
						cv::minMaxIdx(m_depth, &min, &max);
						cv::Mat heat_map, colored;

						m_depth.convertTo(heat_map, CV_8U, 255 / (max - min), -min);
						cv::cvtColor(heat_map, colored, cv::COLOR_GRAY2RGB);
								
						for (size_t i = 0; i < v_users_cord_mm.size(); i+=3)
						{
							float depth_x = 0, depth_y = 0;
							p_to_user_tracker->convertJointCoordinatesToDepth(v_users_cord_mm[i], v_users_cord_mm[i+1], v_users_cord_mm[i+2], &depth_x, &depth_y);
							
							if (!isnan(depth_x) && !isnan(depth_y))
								cv::circle(colored, cv::Point(static_cast<int>(depth_x), static_cast<int>(depth_y)), CIRCLE_SIZE, cv::Scalar(colors[i], colors[i+1], colors[i+2]), CV_FILLED, 8);
						}
						cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE);
						cv::imshow("Depth", colored);
					
						cv::waitKey(1);
					}
					
				}

				if (visualize)
					cv::destroyWindow("Depth");
			}
			else 
			{
				std::cout << "Can't acquire from depth sensor. Exit" << std::endl;
				break;
			}

			std::cout << "ESC pressed, shutdown app.";

			//Delete and udp connection
			delete p_udp_client;

		}
		break;
	}

	//delete user tracker
	delete p_to_user_tracker;

	shutDown(p_to_coord_mapper, p_to_depth_stream, p_to_device);

	//Close Nite
	nite::NiTE::shutdown();

	//Close OpeNI
	openni::OpenNI::shutdown();

}