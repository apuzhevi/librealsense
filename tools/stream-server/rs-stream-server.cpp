// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <clocale>

#if defined(_WIN32)
  #include <windows.h> 
  #include <tchar.h>
  #include <strsafe.h>
#else
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <string.h>
  #include <fcntl.h>
  #include <unistd.h> 
#endif

const int W = 640;
const int H = 480;
const int BPP = 2;

#define FIFO_NAME "/tmp/DDStream"

#if defined(_WIN32)
HANDLE create_named_pipe(std::string stream_name) {
	HANDLE hPipe;

	hPipe = CreateNamedPipeA(
		stream_name.c_str(),
		PIPE_ACCESS_DUPLEX,
		PIPE_TYPE_BYTE| PIPE_WAIT,   // FILE_FLAG_FIRST_PIPE_INSTANCE is not needed but forces CreateNamedPipe(..) to fail if the pipe already exists...
		1,
		1024 * 16,
		1024 * 16,
		NMPWAIT_USE_DEFAULT_WAIT,
		NULL);

	return hPipe;
}
#else
int create_named_pipe(std::string stream_name) {
	std::cout << "INF: Creating pipe - " << stream_name.c_str() << std::endl;

	// create named pipe
	int fifo = mkfifo(stream_name.c_str(), 0666);
	return fifo;
}
#endif

int main(int argc, char* argv[]) {
    try {
	std::cout << "INF: Waiting for connection..." << std::endl;
#if defined(_WIN32)
	HANDLE hDepthPipe = create_named_pipe("\\\\.\\pipe\\DepthStream");
	DWORD dwWritten;
	if (ConnectNamedPipe(hDepthPipe, NULL) != FALSE)   // wait for someone to connect to the pipe
#else
	int fifo = create_named_pipe(FIFO_NAME);
	if ((fifo != 0) && (errno != EEXIST)) {
		std::cout << "ERR: Cannot create pipe - " << std::strerror(errno) << std::endl;
	} else if (fifo != 0) {
		std::cout << "WRN: Pipe exists " << std::endl;
	}

  	int bytes = 0;
	std::cout << "INF: Waiting for connection..." << std::endl;
  	int fd = open(FIFO_NAME, O_WRONLY);
	if (fd != -1) 
#endif			
	{
		std::cout << "INF: Client connected!" << std::endl;

		int ret = fcntl(fd, F_SETPIPE_SZ, 1024*1024);
		if (ret < 0) {
			std::cout << "ERR: Set pipe size - " << std::strerror(errno) << std::endl;
		}

		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline pipe;
		// Start streaming with default recommended configuration
		rs2::config cfg;
		cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
		pipe.start(cfg);

		while (1) {
			rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
			rs2::frame depth = data.get_depth_frame();

			// Query frame size (width and height)

#if defined(_WIN32)
			WriteFile(hDepthPipe, depth.get_data(), H*W*BPP, &dwWritten, NULL);
#else
			bytes = write(fd, depth.get_data(), H*W*BPP);
#endif				
			//std::cout << "Wrote " << dwWritten << " Bytes to buffer!" << std::endl;

		}

#if defined(_WIN32)
		DisconnectNamedPipe(ConnectNamedPipe);
#else
		close(fd);
#endif
		std::cout << "Client discconected!" << std::endl;
		
	} else {
		std::cout << "ERR: Cannot open the pipe - " << std::strerror(errno) << std::endl;
	}
	return EXIT_SUCCESS;
    }
    catch (const rs2::error& e)
    {
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
    }  
    catch (const std::exception& e)
    {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
    }
}

