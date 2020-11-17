// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <vector>


float get_depth_scale(rs2::device dev);
void printPixelDepth(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char* argv[]) try
{
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline pipe;

    //Calling pipeline's start() without any additional parameters will start the first device
// with its default streams.
//The start function returns the pipeline profile which the pipeline used to start the device
    rs2::pipeline_profile profile = pipe.start();

    // Each depth camera might have different units for depth pixels, so we get it here
    // Using the pipeline's profile, we can retrieve the device that the pipeline uses
    float depth_scale = get_depth_scale(profile.get_device());

    //Pipeline could choose a device that does not have a color stream
//If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(profile.get_streams());

    // Create a rs2::align object.
// rs2::align allows us to perform alignment of depth frames to others frames
//The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);


    // Configure and start the pipeline
    //p.start();

    while (true)
    {
        // Block program until frames arrive
        rs2::frameset frameset = pipe.wait_for_frames();

        float depth_scale = get_depth_scale(profile.get_device());

        //Get processed aligned frame
        auto processed = align.process(frameset);

        rs2::video_frame other_frame = processed.first(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        printPixelDepth(other_frame, aligned_depth_frame, depth_scale);

        /*
        for (float i = 1; i < height; i += 4) {
            for (float j = 1; j < width; j += 4) {
                float width = floor(depth.get_width()/i);
                //std::cout << "Type: " << typeid(width).name() << ". Value: " << width <<  std::endl;
                float height = floor(depth.get_height()/j);
               // std::cout << "Type: " << typeid(height).name() << ". Value: " << height << std::endl;
                float dist_to_center = depth.get_distance(width / 2.0, height / 2.0);
                //std::cout << "Position width: " << width << ". Position depth: " << depth << ". Distance : " << dist_to_center << std::endl;
                std::cout << "Depth at [" << i << "," << j << "] is: " << depth.get_distance(j, i) << std::endl;
                Sleep(500);
            }
        }
        */

        /*
        float width = depth.get_width();
        float height = depth.get_height();
        std::cout << "Width: " << width << ". Height: " << height << std::endl;
        */

        /*
        // Get the depth frame's dimensions
        float width = depth.get_width();
        float height = depth.get_height();

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth.get_distance(width / 2, height / 2);

        // Print the distance
        std::cout << "Distance: " << dist_to_center << " in meters" << std::endl;
        //std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
        */

        //Sleep(1000);
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

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}


void printPixelDepth(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale) {

    const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());

    int width = other_frame.get_width();
    int height = other_frame.get_height();
    int other_bpp = other_frame.get_bytes_per_pixel();

    float furthest = 0;
    float closest = 10;

#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Get the depth value of the current pixel
            auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

            if (pixels_distance > furthest) { 
				furthest = pixels_distance;
				//std::cout << "Furthest: " << furthest << "m -- ";
			/*
			if ((x <= 640) && (y <= 240)) {
                std::cout << "Bottom-left qudrant" << std::endl;
            }
            else if ((x > 640 && x <= 1280) && (y < 240)) {
                std::cout << "Bottom-right quadrant" << std::endl;
            }
            else if ((x < 640) && (y > 240 && y <= 480)) {
                std::cout << "Middle-left quadrant" << std::endl;
            }
            else if ((x > 640 && x <= 1280) && (y > 240 && y <= 480)) {
                std::cout << "Middle-right quadrant" << std::endl;
            }
            else if ((x <= 426) && (y > 480 && y <= 720)) {
                std::cout << "Top-left quadrant" << std::endl;
            }
            else if ((x > 427 && x <= 853) && (y > 481 && y <= 720)) {
                std::cout << "Top-middle quadrant" << std::endl;
            }
            else if ((x > 854 && x <= 1280) && (y > 480 && y <= 720)) {
                std::cout << "Top-right quadrant" << std::endl;
            }
            */
				
			}
            if ((pixels_distance < closest) && (pixels_distance > .1)) { 
				closest = pixels_distance; 
				//std::cout << "Closest: " << closest << "m" << std::endl;
			/*
			if ((x <= 640) && (y <= 240)) {
                std::cout << "Bottom-left qudrant" << std::endl;
            }
            else if ((x > 640 && x <= 1280) && (y < 240)) {
                std::cout << "Bottom-right quadrant" << std::endl;
            }
            else if ((x < 640) && (y > 240 && y <= 480)) {
                std::cout << "Middle-left quadrant" << std::endl;
            }
            else if ((x > 640 && x <= 1280) && (y > 240 && y <= 480)) {
                std::cout << "Middle-right quadrant" << std::endl;
            }
            else if ((x <= 426) && (y > 480 && y <= 720)) {
                std::cout << "Top-left quadrant" << std::endl;
            }
            else if ((x > 427 && x <= 853) && (y > 481 && y <= 720)) {
                std::cout << "Top-middle quadrant" << std::endl;
            }
            else if ((x > 854 && x <= 1280) && (y > 480 && y <= 720)) {
                std::cout << "Top-right quadrant" << std::endl;
            }
            */
			}


            //std::cout << "Depth at index (height, width) = [" << y << "," << x << "] = " << pixels_distance << std::endl;


            /*


            if ((x <= 640) && (y <= 240)) {
                std::cout << "Bottom-left qudrant" << std::endl;
            }
            else if ((x > 640 && x <= 1280) && (y < 240)) {
                std::cout << "Bottom-right quadrant" << std::endl;
            }
            else if ((x < 640) && (y > 240 && y <= 480)) {
                std::cout << "Middle-left quadrant" << std::endl;
            }
            else if ((x > 640 && x <= 1280) && (y > 240 && y <= 480)) {
                std::cout << "Middle-right quadrant" << std::endl;
            }
            else if ((x <= 426) && (y > 480 && y <= 720)) {
                std::cout << "Top-left quadrant" << std::endl;
            }
            else if ((x > 427 && x <= 853) && (y > 481 && y <= 720)) {
                std::cout << "Top-middle quadrant" << std::endl;
            }
            else if ((x > 854 && x <= 1280) && (y > 480 && y <= 720)) {
                std::cout << "Top-right quadrant" << std::endl;
            }
            */
        }
    }
 	std::cout << "Furthest: " << furthest << "m. Closest: " << closest << "m." << std::endl;

}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if (!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}
