/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "ros_compat.h"
#include "image_converter.h"
#include <jetson-utils/videoSource.h>


// globals	
videoSource* stream_0 = NULL;
videoSource* stream_1 = NULL;
imageConverter* image_cvt_0 = NULL;
imageConverter* image_cvt_1 = NULL;
Publisher<sensor_msgs::Image> image_pub_0 = NULL;
Publisher<sensor_msgs::Image> image_pub_1 = NULL;
sensor_msgs::Image msg_0;
sensor_msgs::Image msg_1;


// aquire and publish camera frame
bool aquireFrame()
{
	imageConverter::PixelType* nextFrame_0 = NULL;
	imageConverter::PixelType* nextFrame_1 = NULL;

	// get the latest frame
	if( !stream_0->Capture(&nextFrame_0, 1000) )
	{
		ROS_ERROR("failed to capture next frame_0");
		return false;
	}

	// populate timestamp in header field
	msg_0.header.stamp = ROS_TIME_NOW();

	if( !stream_1->Capture(&nextFrame_1, 1000) )
	{
		ROS_ERROR("failed to capture next frame_1");
		return false;
	}

	// populate timestamp in header field
	msg_1.header.stamp = ROS_TIME_NOW();

	// assure correct image sizes
	if( !image_cvt_0->Resize(stream_0->GetWidth(), stream_0->GetHeight(), imageConverter::ROSOutputFormat) )
	{
		ROS_ERROR("failed to resize camera_0 image converter");
		return false;
	}

	if( !image_cvt_1->Resize(stream_1->GetWidth(), stream_1->GetHeight(), imageConverter::ROSOutputFormat) )
	{
		ROS_ERROR("failed to resize camera_1 image converter");
		return false;
	}

	if( !image_cvt_0->Convert(msg_0, imageConverter::ROSOutputFormat, nextFrame_0) )
	{
		ROS_ERROR("failed to convert video stream_0 frame to sensor_msgs::Image");
		return false;
	}

	if( !image_cvt_1->Convert(msg_1, imageConverter::ROSOutputFormat, nextFrame_1) )
	{
		ROS_ERROR("failed to convert video stream_1 frame to sensor_msgs::Image");
		return false;
	}

	// publish the message
	image_pub_0->publish(msg_0);
	ROS_DEBUG("published %ux%u video_0 frame", stream_0->GetWidth(), stream_0->GetHeight());
	image_pub_1->publish(msg_1);
	ROS_DEBUG("published %ux%u video_1 frame", stream_1->GetWidth(), stream_1->GetHeight());
	
	return true;
}


// node main loop
int main(int argc, char **argv)
{
	/*
	 * create node instance
	 */
	ROS_CREATE_NODE("two_video_sources");

	/*
	 * declare parameters
	 */
	videoOptions video_options;

	std::string resource_0 = "csi://0"; 
	std::string resource_1 = "csi://1";
	std::string codec_str; // video type video/x-raw or video/x-jpeg ...
	std::string flip_str;
	
	int video_width = video_options.width;
	int video_height = video_options.height;
    int video_framerate = video_options.frameRate;
	int rtsp_latency = video_options.rtspLatency;
    int num_buffers = video_options.numBuffers;
	
	ROS_DECLARE_PARAMETER("codec", codec_str);
	ROS_DECLARE_PARAMETER("width", video_width);
	ROS_DECLARE_PARAMETER("height", video_height);
	ROS_DECLARE_PARAMETER("framerate", video_framerate);
	ROS_DECLARE_PARAMETER("loop", video_options.loop);
	ROS_DECLARE_PARAMETER("flip", flip_str);
	ROS_DECLARE_PARAMETER("rtsp_latency", rtsp_latency);
    ROS_DECLARE_PARAMETER("num_buffers", num_buffers);
	
	/*
	 * retrieve parameters
	 */
	ROS_GET_PARAMETER("codec", codec_str);
	ROS_GET_PARAMETER("width", video_width);
	ROS_GET_PARAMETER("height", video_height);
	ROS_GET_PARAMETER("framerate", video_framerate);
	ROS_GET_PARAMETER("loop", video_options.loop);
	ROS_GET_PARAMETER("flip", flip_str);
	ROS_GET_PARAMETER("rtsp_latency", rtsp_latency);
    ROS_GET_PARAMETER("num_buffers", num_buffers);

	if( codec_str.size() != 0 )
		video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

	if( flip_str.size() != 0 )
		video_options.flipMethod = videoOptions::FlipMethodFromStr(flip_str.c_str());
	
	video_options.width = video_width;
	video_options.height = video_height;
    video_options.frameRate = video_framerate;
	video_options.rtspLatency = rtsp_latency;
    video_options.numBuffers = num_buffers;
	
	ROS_INFO("opening video sources");

	/*
	 * open video source
	 */
	stream_0 = videoSource::Create(resource_0.c_str(), video_options);
	stream_1 = videoSource::Create(resource_1.c_str(), video_options);

	if( !stream_0 )
	{
		ROS_ERROR("failed to open video source 0");
		return 0;
	}

	if( !stream_1 )
	{
		ROS_ERROR("failed to open video source 1");
		return 0;
	}

	/*
	 * create image converter
	 */
	image_cvt_0 = new imageConverter();
	image_cvt_1 = new imageConverter();

	if( !image_cvt_0 )
	{
		ROS_ERROR("failed to create imageConverter 0");
		return 0;
	}

		if( !image_cvt_1 )
	{
		ROS_ERROR("failed to create imageConverter 1");
		return 0;
	}

	/*
	 * advertise publisher topics
     * ROS_CREATE_PUBLISHER(msgType, topic, queue, ptr)
	 */
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "image_0_raw", 20, image_pub_0);
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "image_1_raw", 20, image_pub_1);

	/*
	 * start the camera streaming
	 */
	if( !stream_0->Open() )
	{
		ROS_ERROR("failed to start streaming video source 0");
		return 0;
	}

	if( !stream_1->Open() )
	{
		ROS_ERROR("failed to start streaming video source 1");
		return 0;
	}

	/*
	 * start publishing video frames
	 */
	while( ROS_OK() )
	{
		if( !aquireFrame() )
		{
			if( !stream_0->IsStreaming() )
			{
				ROS_INFO("stream_0 is closed or reached EOS, exiting node...");
				break;
			}

			if( !stream_1->IsStreaming() )
			{
				ROS_INFO("stream_1 is closed or reached EOS, exiting node...");
				break;
			}
		}

		if( ROS_OK() )
			ROS_SPIN_ONCE();
	}


	/*
	 * free resources
	 */
	delete stream_0;
	delete stream_1;
	delete image_cvt_0;
	delete image_cvt_1;

	return 0;
}

