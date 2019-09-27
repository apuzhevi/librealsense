// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <windows.h> 
#include <stdio.h>
#include <tchar.h>
#include <strsafe.h>

const int W = 640;
const int H = 480;
const int BPP = 2;

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
int main(int argc, char* argv[]) try
{

		std::cout << "Waiting for connection...";
		HANDLE hDepthPipe = create_named_pipe("\\\\.\\pipe\\DepthStream");
		DWORD dwWritten;
		if (ConnectNamedPipe(hDepthPipe, NULL) != FALSE)   // wait for someone to connect to the pipe
		{
			std::cout << "Client connected!" << std::endl;
			// Declare RealSense pipeline, encapsulating the actual device and sensors
			rs2::pipeline pipe;
			// Start streaming with default recommended configuration
			rs2::config cfg;
			cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
			pipe.start(cfg);

			while (1)
			{
				rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
				rs2::frame depth = data.get_depth_frame();

				// Query frame size (width and height)

				WriteFile(hDepthPipe, depth.get_data(), H*W*BPP, &dwWritten, NULL);
				//std::cout << "Wrote " << dwWritten << " Bytes to buffer!" << std::endl;

			}

			DisconnectNamedPipe(ConnectNamedPipe);
			std::cout << "Client discconected!" << std::endl;
		
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
