#include <windows.h>

#include <iostream>
#include <fstream>
#include <condition_variable>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#define BUFFERSIZE 21
DWORD g_BytesTransferred = 0;

VOID CALLBACK FileIOCompletionRoutine(__in  DWORD dwErrorCode, __in  DWORD dwNumberOfBytesTransfered, __in  LPOVERLAPPED lpOverlapped)
{
	g_BytesTransferred = dwNumberOfBytesTransfered;
}


time_t readLastCascadeTime(const std::string fn_log)
{
	time_t time;

	HANDLE hFile;
	DWORD  dwBytesRead = 0;
	char   buffer_read[BUFFERSIZE] = { 0 };
	OVERLAPPED ol = { 0 };

	hFile = CreateFile(fn_log.c_str(),               // file to open
		GENERIC_READ,          // open for reading
		FILE_SHARE_READ,       // share for reading
		NULL,                  // default security
		OPEN_EXISTING,         // existing file only
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, // normal file
		NULL);                 // no attr. template

	if (hFile == INVALID_HANDLE_VALUE)
	{
		return 0;
	}

	if (FALSE == ReadFileEx(hFile, buffer_read, BUFFERSIZE - 1, &ol, FileIOCompletionRoutine))
	{
		printf("Terminal failure: Unable to read from file.\n GetLastError=%08x\n", GetLastError());
		CloseHandle(hFile);
		return 0;
	}

	SleepEx(5000, TRUE);
	dwBytesRead = g_BytesTransferred;
	// This is the section of code that assumes the file is ANSI text. 
	// Modify this block for other data types if needed.

	if (dwBytesRead > 0 && dwBytesRead <= BUFFERSIZE - 1)
	{
		buffer_read[dwBytesRead] = '\0';
		const std::string read(buffer_read);

		std::vector<std::string> v_split_string;
		boost::split(v_split_string, read, boost::is_any_of("-"));

		tm last_alive_time = { 0 };
		last_alive_time.tm_year = boost::lexical_cast<int>(v_split_string[0]) - 1900; /* years since 1900 */
		last_alive_time.tm_mon = boost::lexical_cast<int>(v_split_string[1]) - 1;
		last_alive_time.tm_mday = boost::lexical_cast<int>(v_split_string[2]);
		last_alive_time.tm_hour = boost::lexical_cast<int>(v_split_string[3]);
		last_alive_time.tm_min = boost::lexical_cast<int>(v_split_string[4]);
		last_alive_time.tm_sec = boost::lexical_cast<int>(v_split_string[5]);
		last_alive_time.tm_isdst = 1;

		time = mktime(&last_alive_time); 
	}
	else if (dwBytesRead == 0)
	{
		//Kinect is not writing
		time = 0;
	}
	else
	{
		printf("\n ** Unexpected value for dwBytesRead ** \n");
		time = 0;
	}

	CloseHandle(hFile);

	return time;
}

int main(int argc, char *argv[]) 
{
	//time in millisecond before start cascade
	const int seconds_to_wait_start_up = 3;
	const int seconds_to_wait_watchdog = 20;
	const int seconds_to_restart = 1 * 60;

	std::string fn_cascade_time = "C:\\log\\log.txt";
	std::string path_to_exe = "C:\\workspace\\cascade\\build\\Release\\cascade.exe 1";
	const std::string path_to_cascade = "C:\\workspace\\cascade\\build\\";

	printf("Kinect slave is up, system will start in %d seconds \n", seconds_to_wait_start_up);
	
	Sleep(seconds_to_wait_start_up * 1000);
	
	//Start application
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));

	// Start the child process. 
	LPSTR name_exe = const_cast<char *>(path_to_exe.c_str());
	LPSTR dir_cascade = const_cast<char *>(path_to_cascade.c_str());

	if (!CreateProcess(NULL, name_exe, NULL, NULL, FALSE, 0, NULL, dir_cascade, &si, &pi))
	{
		printf("CreateProcess failed (%d).\n", GetLastError());
		return 0;
	}
	else 
	{
		printf("Cascade is running with PID: %d \n", pi.dwProcessId);
	}

	//start log
	time_t now = std::time(0);
	tm *local_time = std::localtime(&now);

	std::ostringstream ss_time;
	ss_time << std::put_time(local_time, "%d-%m-%y");

	//date for log filename
	const std::string date = ss_time.str();
	//hour
	std::string hour;

	std::string fn_log = "C://log//" + date + ".log";
	std::ofstream *p_file_log = new std::ofstream(fn_log, std::ios::app);

	std::mutex mutex;
	std::condition_variable condition;

	std::chrono::duration<int, std::milli> time_out(seconds_to_wait_watchdog * 1000);

	std::unique_lock<std::mutex> lock (mutex);

	while (true) 
	{
		//Check every time out seconds
		condition.wait_for(lock, time_out);

		time_t last_cascade_time = readLastCascadeTime(fn_cascade_time);

		//clear stream
		ss_time.str("");

		//Get date
		now = std::time(0);
		local_time = std::localtime(&now);

		ss_time << std::put_time(local_time, "%H-%M-%S");
		hour = ss_time.str();

		if(hour[0] == '0' && hour[1]=='0')
		{
			//close old file
			p_file_log->close();
			delete p_file_log;

			//date is changed
			ss_time.str("");
			ss_time << std::put_time(local_time, "%d-%m-%y");

			p_file_log = new std::ofstream("C://log//" + ss_time.str() + ".log", std::ios::app);

			ss_time.str("");
			ss_time << std::put_time(local_time, "%H-%M-%S");
			hour = ss_time.str();
		}

		tm *timec = std::localtime(&last_cascade_time);

		double seconds_diff = difftime(now, last_cascade_time);

		if (seconds_diff < seconds_to_restart)
		{
			*p_file_log << "[RUNNING]" << hour << std::endl;
		}
		else 
		{
			//Kinect is not running shutdown
			*p_file_log << "[RESTART]" << hour << std::endl;

			// Close process and thread handles. 
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);

			//now restart
			p_file_log->close();
			delete p_file_log;

			system("shutdown -r");
		}
	}

	return 0;
}