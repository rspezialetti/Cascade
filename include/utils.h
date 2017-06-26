#ifndef UTILS_H
#define UTILS_H

#include <sstream>
#include <string>
#include <map>
#include <numeric>

#include <Windows.h>


#define ERROR_MESSAGE(msg) {printf("Error in command arguments: %s",msg);}

#define ERROR_CALIBRATION_MESSAGE(msg) {printf("Error in CALIBRATION: %s",msg);}

int readCmdLine(int argc, char** argv, int& mode, std::string& kinect_id);

int readConfigurationFile(const std::string file_name, std::map<std::string, std::string>& v_values);

inline float calculateSquaredEuclideanDistances(float x1, float y1, float z1, float x2, float y2, float z2)
{
	float dist = 0;

	float dx = x1 - x2;
	float dy = y1 - y2;
	float dz = z1 - z2;

	dist = dx * dx + dy * dy + dz * dz;      //calculating Euclidean distance

	return  dist;
}

int writeTimestamp(const std::string fn_timestamp, const time_t& time_stamp);


template <typename type>
double calculateSTDV(std::vector<type>& values)
{
	int n_el = static_cast<int>(values.size());
	double sum = std::accumulate(values.begin(), values.end(), 0);
	double mean = sum / n_el;

	double acc = 0;
	std::for_each(values.begin(), values.end(), [&](const type d) {	acc += (d - mean) * (d - mean);	});

	return std::sqrt(acc / n_el);
}

inline float checkUserMovement(std::vector<float>& values)
{
	std::vector<float> last_position{ values[values.size() - 3], values[values.size() - 2], values[values.size() - 1] };
	std::vector<float> mean(3, 0.f);

	for (size_t i_p = 0; i_p < values.size() - 3; i_p+=3)
	{
		mean[0] += values[i_p];
		mean[1] += values[i_p+1];
		mean[2] += values[i_p+2];
	}
	const int number_of_points = static_cast<int>(values.size()) / 3;
	mean[0] /= number_of_points;
	mean[1] /= number_of_points;
	mean[2] /= number_of_points;

	return std::sqrt(calculateSquaredEuclideanDistances(last_position[0], last_position[1], last_position[2], mean[0], mean[1], mean[2]));
}

#endif //UTILS_H
