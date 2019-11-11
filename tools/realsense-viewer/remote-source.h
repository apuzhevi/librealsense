// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once

#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <functional>
#include <thread>

#if defined(_WIN32)
  #include <windows.h> 
  #include <stdio.h>
  #include <tchar.h>
  #include <strsafe.h>
#else
  #include <unistd.h>
  #include <sys/stat.h>
  #include <sys/ioctl.h>
  #include <fcntl.h>
#endif

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/hpp/rs_internal.hpp>
#include <mutex>

#if 1
const int W = 640;
const int H = 480;
const int BPP = 2;
#else
const int W = 1280;
const int H = 720;
const int BPP = 2;
#endif

namespace rs2
{
	void deleter(void* p) {

	}
	class remote_frame_source
	{

	public:
		remote_frame_source()
		{


			depth_frame.bpp = BPP;
			depth_frame.stride = BPP * W;
			pixels.resize(depth_frame.stride * H, 0);
			depth_frame.pixels = pixels.data();
			dev = rs2_create_software_device(NULL);
			depth_frame.deleter = &deleter;
			create_sensors();

		}
		virtual ~remote_frame_source() {
#if defined(_WIN32)			
			CloseHandle(hPipe);
#else
			close(fd);
#endif
			rs2_delete_device(dev);

		}


		rs2_intrinsics get_intrinsics()
		{
			rs2_intrinsics intrinsics = { W, H,
				(float)W / 2, (float)H / 2,
				(float)W / 2, (float)H / 2,
				RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

			return intrinsics;
		}
		void start(std::string address, std::string stream_name) {
			if (is_active)
				return;
#if defined(_WIN32)
			hPipe = CreateFileA(
				stream_name.c_str(),
				GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
#else
			int fifo = mkfifo(stream_name.c_str(), 0666);
			fd = open(stream_name.c_str(), O_RDONLY);
			int ret = fcntl(fd, F_SETPIPE_SZ, 1024*1024);
			if (ret < 0) {
				std::cout << "ERR: Set pipe size - " << std::strerror(errno) << std::endl;
			}
#endif			
			t = std::thread{ &remote_frame_source::thread_main, this };
			is_active = true;
		}
		void stop() {
			if (!is_active)
				return;
			is_active = false;
			t.join();
#if defined(_WIN32)			
			CloseHandle(hPipe);
#else
			close(fd);
#endif
		}

		rs2_software_video_frame& get_frame() {
			std::lock_guard<std::mutex> lck(mtx);
			return depth_frame;
		}

		rs2_device* get_device() {
			return dev;
		}

	private:
		void create_sensors() {
			rs2_intrinsics depth_intrinsics = get_intrinsics();
			depth_sensor = rs2_software_device_add_sensor(dev, "Depth (Remote)", NULL);
			rs2_video_stream st = { RS2_STREAM_DEPTH, 0, 1, W,
									H, 30, BPP,
									RS2_FORMAT_Z16, depth_intrinsics };
			depth_stream = rs2_software_sensor_add_video_stream(depth_sensor, st, NULL);
			depth_frame.profile = depth_stream;
		}
		void read_frame()
		{
			std::lock_guard<std::mutex> lck(mtx);
#if defined(_WIN32)
			{
				DWORD bytesRead = 0;
				if (ReadFile(hPipe, depth_frame.pixels, W * H * BPP, &bytesRead, NULL)) {
#else
			int nbytes = 0;
			ioctl(fd, FIONREAD, &nbytes);
			if (nbytes >= W*H*BPP) {
				int bytesRead = read(fd, depth_frame.pixels, W * H * BPP);
				if (bytesRead != -1) {
#endif				
					std::cout << " Read: " << bytesRead << " Bytes from buffer!" << std::endl;
				}
				else
					std::cerr << "Cannot read from buffer!" << std::endl;

				using namespace std::chrono;
				auto now = system_clock::now();
				depth_frame.timestamp = time_point_cast<milliseconds>(now).time_since_epoch().count();
				rs2_software_sensor_on_video_frame(depth_sensor, depth_frame, NULL);
	
				depth_frame.frame_number++;
			}

		}
		void thread_main() {
			while (is_active) {
				read_frame();
			}
		}
		bool is_active = false;

#if defined(_WIN32)
		HANDLE hPipe;
#else
		int fifo;
		int fd;
#endif
		int frame_number = 0;
		std::chrono::high_resolution_clock::time_point last;
		rs2_device* dev; // Create software-only device
		rs2_sensor* depth_sensor;
		rs2_stream_profile* depth_stream;
		rs2_software_video_frame depth_frame;
		std::vector<uint8_t> pixels;
		std::thread t;
		std::mutex mtx;

	};



}
