#ifndef UTILS_CPP_IMPL
#define UTILS_CPP_IMPL

#include <iomanip>
#include <fstream>
#include <vector>

#include <boost/algorithm/string.hpp>

#include "utils.h"

int readCmdLine(int argc, char** argv, int& mode, std::string& kinect_id)
{
	switch (argc)
	{
	case(2) :
	{
		try
		{
			mode = std::stoi(argv[1]);
		}
		catch (std::exception const & e)
		{
			ERROR_MESSAGE(e.what());
			return -1;
		}
	}
			break;

	case(3) :
	{
		try
		{
			mode = std::stoi(argv[1]);
			kinect_id = argv[2];
		}
		catch (std::exception const & e)
		{
			ERROR_MESSAGE(e.what());
			return -1;
		}
	}
			break;
	default:
		ERROR_MESSAGE("Usage is mode: [0-calibration | 1-tracking] kinect id: [empty = any device]");
		return -1;
		break;
	}

	return 1;
}


int readConfigurationFile(const std::string file_name, std::map<std::string, std::string>& v_values) 
{
	std::fstream file_config(file_name);
	int count = 0;

	if(file_config.is_open())
	{
		std::string line;
	
		while (std::getline(file_config, line))
		{
			if (count == 0 || line =="" || (std::find(line.begin(), line.end(), '#') != line.end())) 
			{
				count++;
				continue;
			}

			std::vector<std::string> v_splitted;
			boost::split(v_splitted, line, boost::is_any_of(":"));
			v_values.insert(std::pair<std::string, std::string>(v_splitted[0], v_splitted[1]));
		}
	
	}
	file_config.close();

	return count;
}

int writeTimestamp(const std::string fn_timestamp, const time_t& time_stamp)
{
	HANDLE hFile;

	DWORD dwBytesWritten = 0;
	BOOL bErrorFlag = FALSE;

	tm *time_local = std::localtime(&time_stamp);

	std::ostringstream ss_date;
	ss_date << std::put_time(time_local, "%Y-%m-%d-%H-%M-%S");

	const std::string date = ss_date.str();

	hFile = CreateFile(fn_timestamp.c_str(),                // name of the write
		GENERIC_WRITE,          // open for writing
		FILE_SHARE_READ,       //share read
		NULL,                   // default security
		OPEN_ALWAYS,             // create new file only
		FILE_ATTRIBUTE_NORMAL,  // normal file
		NULL);                  // no attr. template

	if (hFile == INVALID_HANDLE_VALUE)
	{
		return 0;
	}

	const char *data = date.c_str();
	DWORD dw_bytes_to_write = (DWORD)strlen(data);

	bErrorFlag = WriteFile(
		hFile,           // open file handle
		data,      // start of data to write
		dw_bytes_to_write,  // number of bytes to write
		&dwBytesWritten, // number of bytes that were written
		NULL);            // no overlapped structure

	if (FALSE == bErrorFlag)
	{
		printf("Terminal failure: Unable to write to file.\n");
	}
	else
	{
		if (dwBytesWritten != dw_bytes_to_write)
		{
			// This is an error because a synchronous write that results in
			// success (WriteFile returns TRUE) should write all data as
			// requested. This would not necessarily be the case for
			// asynchronous writes.
			printf("Error: dwBytesWritten != dwBytesToWrite\n");
		}
	}
	CloseHandle(hFile);

	return dwBytesWritten;
}



#endif //UTILS_CPP