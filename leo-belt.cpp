// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include "common/example.hpp"
#include <imgui.h>
#include "imgui_impl_glfw.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "rs232.h"

#define BUF_SIZE 123
char str_send[56][BUF_SIZE]; // send data buffer
int cport_nr = 16;

float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
void printPixelDepth(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale);
void silenceAllFeathers();

int main(int argc, char * argv[]) try
{
    int bdrate = 57600;
    
    char mode[] ={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
    unsigned char str_recv[BUF_SIZE]; // recv data buffer
    
    // Strings that will be sent over serial to the Master Feather
    // Format is "Intensity-Feather#", so "01" would be 0 intensity sent to Feather 1
    // A translates to intensity 1, B to 2, C to 3, D to 4, E to 5, F to 6
    strcpy(str_send[0], "01"); strcpy(str_send[1], "A1");
    strcpy(str_send[2], "B1"); strcpy(str_send[3], "C1");
    strcpy(str_send[4], "D1"); strcpy(str_send[5], "E1");
    strcpy(str_send[6], "F1"); strcpy(str_send[7], "02");
    strcpy(str_send[8], "A2"); strcpy(str_send[9], "B2");
    strcpy(str_send[10], "C2"); strcpy(str_send[11], "D2");
    strcpy(str_send[12], "E2"); strcpy(str_send[13], "F2");
    strcpy(str_send[14], "03"); strcpy(str_send[15], "A3");
    strcpy(str_send[16], "B3"); strcpy(str_send[17], "C3");
    strcpy(str_send[18], "D3"); strcpy(str_send[19], "E3");
    strcpy(str_send[20], "F3"); strcpy(str_send[21], "04");
    strcpy(str_send[22], "A4"); strcpy(str_send[23], "B4");
    strcpy(str_send[24], "C4"); strcpy(str_send[25], "D4");
    strcpy(str_send[26], "E4"); strcpy(str_send[27], "F4");
    strcpy(str_send[28], "05"); strcpy(str_send[29], "A5");
    strcpy(str_send[30], "B5"); strcpy(str_send[31], "C5");
    strcpy(str_send[32], "D5"); strcpy(str_send[33], "E5");
    strcpy(str_send[34], "F5"); strcpy(str_send[35], "06");
    strcpy(str_send[36], "A6"); strcpy(str_send[37], "B6");
    strcpy(str_send[38], "C6"); strcpy(str_send[39], "D6");
    strcpy(str_send[40], "E6"); strcpy(str_send[41], "F6");
    strcpy(str_send[42], "07"); strcpy(str_send[43], "A7");
    strcpy(str_send[44], "B7"); strcpy(str_send[45], "C7");
    strcpy(str_send[46], "D7"); strcpy(str_send[47], "E7");
    strcpy(str_send[48], "F7"); strcpy(str_send[49], "08");
    strcpy(str_send[50], "A8"); strcpy(str_send[51], "B8");
    strcpy(str_send[52], "C8"); strcpy(str_send[53], "D8");
    strcpy(str_send[54], "E8"); strcpy(str_send[55], "F8");
    
    if(RS232_OpenComport(cport_nr, bdrate, mode)) {
        printf("Can not open comport\n");
        return(0);
    }
    
    usleep(2000000); // Waits 2000ms for stable condition
    
    // Create and initialize GUI related objects
    window app(1280, 720, "LEO Belt"); // Simple window handling
    ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition
    rs2::colorizer c;                     // Helper to colorize depth images
    texture renderer;                     // Helper for renderig images

    // Create a pipeline to easily configure and start the camera
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

    while (app) // Application still alive?
    {
        // Using the align object, we block the application until a frameset is available
        rs2::frameset frameset = pipe.wait_for_frames();

        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
        {
            //If the profile was changed, update the align object, and also get the new device's depth scale
            profile = pipe.get_active_profile();
            align_to = find_stream_to_align(profile.get_streams());
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }

        //Get processed aligned frame
        auto processed = align.process(frameset);

        // Trying to get both other and aligned depth frames
        rs2::video_frame other_frame = processed.first(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        //If one of them is unavailable, continue iteration
        if (!aligned_depth_frame || !other_frame)
        {
            continue;
        }
  
        printPixelDepth(other_frame, aligned_depth_frame, depth_scale);
        
        // Taking dimensions of the window for rendering purposes
        float w = static_cast<float>(app.width());
        float h = static_cast<float>(app.height());

        

        // At this point, "other_frame" is an altered frame, stripped form its background
        // Calculating the position to place the frame in the window
        rect altered_other_frame_rect{ 0, 0, w, h };
        altered_other_frame_rect = altered_other_frame_rect.adjust_ratio({ static_cast<float>(other_frame.get_width()),static_cast<float>(other_frame.get_height()) });

        // Render aligned image
        renderer.render(other_frame, altered_other_frame_rect); 

        // Using ImGui library to provide a slide controller to select the depth clipping distance
        ImGui_ImplGlfw_NewFrame(1);
        ImGui::Render();
    }
    silenceAllFeathers();
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void silenceAllFeathers(){
    RS232_cputs(cport_nr, str_send[0]);
    RS232_cputs(cport_nr, str_send[7]);
    RS232_cputs(cport_nr, str_send[14]);
    RS232_cputs(cport_nr, str_send[21]);
    RS232_cputs(cport_nr, str_send[28]);
    RS232_cputs(cport_nr, str_send[35]);
    RS232_cputs(cport_nr, str_send[42]);
    RS232_cputs(cport_nr, str_send[49]);
}

void printPixelDepth(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale) {

    const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
    
    uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

    int width = other_frame.get_width();
    int height = other_frame.get_height();
    int other_bpp = other_frame.get_bytes_per_pixel();

    int intensities[8] = {0,0,0,0,0,0,0,0};
    float closest[8] = {10,10,10,10,10,10,10,10};

#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Get the depth value of the current pixel
            auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];
            
            auto offset = depth_pixel_index*other_bpp;
            
            // Coloring Segment Borders
            if ((x == floor(width/3)) && (y < 2*(height/3))){
                std::memset(&p_other_frame[offset], 0xFF, other_bpp);
            }
            if ((x == 2*floor(width/3))&& (y < 2*(height/3))){
                std::memset(&p_other_frame[offset], 0xFF, other_bpp);
            }
            if ((x == floor(width/2)) && (y > 2*(height/3))){
                std::memset(&p_other_frame[offset], 0xFF, other_bpp);
            }
            if ((y == floor(height/3))){
                std::memset(&p_other_frame[offset], 0xFF, other_bpp);
            }
            if ((y == 2*floor(height/3))){
                std::memset(&p_other_frame[offset], 0xFF, other_bpp);
            }
            
            // Sending signals to feathers
            if ((x < width / 3) && (y < height / 3)) { // Top-left quadrant -- Feather 1
                if (pixels_distance < closest[0] && pixels_distance > .001){
                    closest[0] = pixels_distance;
                }
            } else if ((x > width / 3) && (x < 2*(width / 3)) && (y < height / 3)) { // Top-middle quadrant -- Feather 2
                if (pixels_distance < closest[1] && pixels_distance > .001){
                    closest[1] = pixels_distance;
                }
            } else if ((x > 2*(width / 3)) && (y < height / 3)) { // Top-right quadrant -- Feather 3
                if (pixels_distance < closest[2] && pixels_distance > .001){
                    closest[2] = pixels_distance;
                }
            } else if ((x < width / 3) && (y > height / 3) && (y < 2*(height / 3))) { // Middle-left quadrant -- Feather 4
                if (pixels_distance < closest[3] && pixels_distance > .001){
                    closest[3] = pixels_distance;
                }
            } else if ((x > width / 3) && (x < 2*(width / 3)) && (y > height / 3) && (y < 2*(height / 3))) { // Middle-middle quadrant -- Feather 5
                if (pixels_distance < closest[4] && pixels_distance > .001){
                    closest[4] = pixels_distance;
                }
            } else if ((x > 2*(width / 3)) && (y > (height / 3)) && (y < (2*(height / 3)))) { // Middle-right quadrant -- Feather 6
                if (pixels_distance < closest[5] && pixels_distance > .001){
                    closest[5] = pixels_distance;
                }
            } else if ((x < width / 2) && (y > 2*(height / 3))) { // Bottom-left quadrant -- Feather 7 
                if (pixels_distance < closest[6] && pixels_distance > .001){
                    closest[6] = pixels_distance;
                }
            } else if ((x > width / 2) && (y > 2*(height / 3))) { // Bottom-right quadrant -- Feather 8 
                if (pixels_distance < closest[7] && pixels_distance > .001){
                    closest[7] = pixels_distance;
                }
            }
        }
    }
    //Feather 1
    //std::cout << "Quadrant 1 closest: " << closest[0] << "m -> Intensity: ";
    //printf("Quadrant 1 closest: %.3fm Intensity: ", closest[0]);
    if(closest[0] < .33){
        intensities[0] = 6;
        RS232_cputs(cport_nr, str_send[6]);
        //std::cout << "6/6" << std::endl;
    } else if(closest[0] < .66){
        intensities[0] = 5;
        RS232_cputs(cport_nr, str_send[5]);
        //std::cout << "5/6" << std::endl;
    } else if(closest[0] < .99){
        intensities[0] = 4;
        RS232_cputs(cport_nr, str_send[4]);
        //std::cout << "4/6" << std::endl;
    } else if(closest[0] < 1.33){
        intensities[0] = 3;
        RS232_cputs(cport_nr, str_send[3]);
        //std::cout << "3/6" << std::endl;
    } else if(closest[0] < 1.66){
        intensities[0] = 2;
        RS232_cputs(cport_nr, str_send[2]);
        //std::cout << "2/6" << std::endl;
    } else if(closest[0] < 1.99){
        intensities[0] = 1;
        RS232_cputs(cport_nr, str_send[1]);
        //std::cout << "1/6" << std::endl;
    } else {
        intensities[0] = 0;
        RS232_cputs(cport_nr, str_send[0]);
        //std::cout << "0/6" << std::endl;
    }
    
    //Feather 2
    //printf("Quadrant 2 closest: %.3fm Intensity: ", closest[1]);
    if(closest[1] < .33){
        intensities[1] = 6;
        RS232_cputs(cport_nr, str_send[13]);
        //std::cout << "6/6\r" << std::endl;
    } else if(closest[1] < .66){
        intensities[1] = 5;
        RS232_cputs(cport_nr, str_send[12]);
        //std::cout << "5/6\r" << std::endl;
    } else if(closest[1] < .99){
        intensities[1] = 4;
        RS232_cputs(cport_nr, str_send[11]);
        //std::cout << "4/6\r" << std::endl;
    } else if(closest[1] < 1.33){
        intensities[1] = 3;
        RS232_cputs(cport_nr, str_send[10]);
        //std::cout << "3/6\r" << std::endl;
    } else if(closest[1] < 1.66){
        intensities[1] = 2;
        RS232_cputs(cport_nr, str_send[9]);
        //std::cout << "2/6\r" << std::endl;
    } else if(closest[1] < 1.99){
        intensities[1] = 1;
        RS232_cputs(cport_nr, str_send[8]);
        //std::cout << "1/6\r" << std::endl;
    } else {
        intensities[1] = 0;
        RS232_cputs(cport_nr, str_send[7]);
        //std::cout << "0/6\r" << std::endl;
    }
    
    //Feather 3
    //printf("Quadrant 3 closest: %.3fm Intensity: ", closest[2]);
    if(closest[2] < .33){
        intensities[2] = 6;
        RS232_cputs(cport_nr, str_send[20]);
        //std::cout << "6/6" << std::endl;
    } else if(closest[2] < .66){
        intensities[2] = 5;
        RS232_cputs(cport_nr, str_send[19]);
        //std::cout << "5/6" << std::endl;
    } else if(closest[2] < .99){
        intensities[2] = 4;
        RS232_cputs(cport_nr, str_send[18]);
        //std::cout << "4/6" << std::endl;
    } else if(closest[2] < 1.33){
        intensities[2] = 3;
        RS232_cputs(cport_nr, str_send[17]);
        //std::cout << "3/6" << std::endl;
    } else if(closest[2] < 1.66){
        intensities[2] = 2;
        RS232_cputs(cport_nr, str_send[16]);
        //std::cout << "2/6" << std::endl;
    } else if(closest[2] < 1.99){
        intensities[2] = 1;
        RS232_cputs(cport_nr, str_send[15]);
        //std::cout << "1/6" << std::endl;
    } else {
        intensities[2] = 0;
        RS232_cputs(cport_nr, str_send[14]);
        //std::cout << "0/6" << std::endl;
    }
    
    //Feather 4
    //printf("Quadrant 4 closest: %.3fm Intensity: ", closest[3]);
    if(closest[3] < .33){
        intensities[3] = 6;
       RS232_cputs(cport_nr, str_send[27]);
        //std::cout << "6/6" << std::endl;
    } else if(closest[3] < .66){
        intensities[3] = 5;
       RS232_cputs(cport_nr, str_send[26]);
        //std::cout << "5/6" << std::endl;
    } else if(closest[3] < .99){
        intensities[3] = 4;
       RS232_cputs(cport_nr, str_send[25]);
        //std::cout << "4/6" << std::endl;
    } else if(closest[3] < 1.33){
        intensities[3] = 3;
       RS232_cputs(cport_nr, str_send[24]);
        //std::cout << "3/6" << std::endl;
    } else if(closest[3] < 1.66){
        intensities[3] = 2;
       RS232_cputs(cport_nr, str_send[23]);
        //std::cout << "2/6" << std::endl;
    } else if(closest[3] < 1.99){
        intensities[3] = 1;
       RS232_cputs(cport_nr, str_send[22]);
        //std::cout << "1/6" << std::endl;
    } else {
        intensities[3] = 0;
       RS232_cputs(cport_nr, str_send[21]);
        //std::cout << "0/6" << std::endl;
    }
    
    //Feather 5
    //printf("Quadrant 5 closest: %.3fm Intensity: ", closest[4]);
    if(closest[4] < .33){
        intensities[4] = 6;
       RS232_cputs(cport_nr, str_send[34]);
        //std::cout << "6/6" << std::endl;
    } else if(closest[4] < .66){
        intensities[4] = 5;
       RS232_cputs(cport_nr, str_send[33]);
        //std::cout << "5/6" << std::endl;
    } else if(closest[4] < .99){
        intensities[4] = 4;
       RS232_cputs(cport_nr, str_send[32]);
        //std::cout << "4/6" << std::endl;
    } else if(closest[4] < 1.33){
        intensities[4] = 3;
       RS232_cputs(cport_nr, str_send[31]);
        //std::cout << "3/6" << std::endl;
    } else if(closest[4] < 1.66){
        intensities[4] = 2;
       RS232_cputs(cport_nr, str_send[30]);
        //std::cout << "2/6" << std::endl;
    } else if(closest[4] < 1.99){
        intensities[4] = 1;
       RS232_cputs(cport_nr, str_send[29]);
        //std::cout << "1/6" << std::endl;
    } else {
        intensities[4] = 0;
       RS232_cputs(cport_nr, str_send[28]);
        //std::cout << "0/6" << std::endl;
    }
    
    //Feather 6
    //printf("Quadrant 6 closest: %.3fm Intensity: ", closest[5]);
    if(closest[5] < .33){
        intensities[5] = 6;
       RS232_cputs(cport_nr, str_send[41]);
        //std::cout << "6/6" << std::endl;
    } else if(closest[5] < .66){
        intensities[5] = 5;
       RS232_cputs(cport_nr, str_send[40]);
        //std::cout << "5/6" << std::endl;
    } else if(closest[5] < .99){
        intensities[5] = 4;
       RS232_cputs(cport_nr, str_send[39]);
        //std::cout << "4/6" << std::endl;
    } else if(closest[5] < 1.33){
        intensities[5] = 3;
       RS232_cputs(cport_nr, str_send[38]);
        //std::cout << "3/6" << std::endl;
    } else if(closest[5] < 1.66){
        intensities[5] = 2;
       RS232_cputs(cport_nr, str_send[37]);
        //std::cout << "2/6" << std::endl;
    } else if(closest[5] < 1.99){
        intensities[5] = 1;
       RS232_cputs(cport_nr, str_send[36]);
        //std::cout << "1/6" << std::endl;
    } else {
        intensities[5] = 0;
       RS232_cputs(cport_nr, str_send[35]);
        //std::cout << "0/6" << std::endl;
    }
    
    //Feather 7
    //printf("Quadrant 7 closest: %.3fm Intensity: ", closest[6]);
    if(closest[6] < .33){
        intensities[6] = 6;
       RS232_cputs(cport_nr, str_send[48]);
        //std::cout << "6/6" << std::endl;
    } else if(closest[6] < .66){
        intensities[6] = 5;
       RS232_cputs(cport_nr, str_send[47]);
        //std::cout << "5/6" << std::endl;
    } else if(closest[6] < .99){
        intensities[6] = 4;
       RS232_cputs(cport_nr, str_send[46]);
        //std::cout << "4/6" << std::endl;
    } else if(closest[6] < 1.33){
        intensities[6] = 3;
       RS232_cputs(cport_nr, str_send[45]);
        //std::cout << "3/6" << std::endl;
    } else if(closest[6] < 1.66){
        intensities[6] = 2;
       RS232_cputs(cport_nr, str_send[44]);
        //std::cout << "2/6" << std::endl;
    } else if(closest[6] < 1.99){
        intensities[6] = 1;
       RS232_cputs(cport_nr, str_send[43]);
        //std::cout << "1/6" << std::endl;
    } else {
        intensities[6] = 0;
       RS232_cputs(cport_nr, str_send[42]);
        //std::cout << "0/6" << std::endl;
    }
    
    //Feather 8
    //printf("Quadrant 8 closest: %.3fm Intensity: ", closest[7]);
    if(closest[7] < .33){
        intensities[7] = 6;
       RS232_cputs(cport_nr, str_send[55]);
        //std::cout << "6/6" << std::endl;
    } else if(closest[7] < .66){
        intensities[7] = 5;
       RS232_cputs(cport_nr, str_send[54]);
        //std::cout << "5/6" << std::endl;
    } else if(closest[7] < .99){
        intensities[7] = 4;
       RS232_cputs(cport_nr, str_send[53]);
        //std::cout << "4/6" << std::endl;
    } else if(closest[7] < 1.33){
        intensities[7] = 3;
       RS232_cputs(cport_nr, str_send[52]);
        //std::cout << "3/6" << std::endl;
    } else if(closest[7] < 1.66){
        intensities[7] = 2;
       RS232_cputs(cport_nr, str_send[51]);
        //std::cout << "2/6" << std::endl;
    } else if(closest[7] < 1.99){
        intensities[7] = 1;
       RS232_cputs(cport_nr, str_send[50]);
        //std::cout << "1/6" << std::endl;
    } else {
        intensities[7] = 0;
       RS232_cputs(cport_nr, str_send[49]);
        //std::cout << "0/6" << std::endl;
    }
    
    
    // Drawing intensities for each feather
    #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Get the depth value of the current pixel
            auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];
            
            auto offset = depth_pixel_index*other_bpp;
            
            // Displaying Feather Intensity
            // Feather 1
            if (intensities[0] > 0){
                if((x > 0) && (x < (20)) && (y < height/3 ) && (y > (height/3 - 30))){
                   std::memset(&p_other_frame[offset], 0xB0, other_bpp); 
                }
            }
            if (intensities[0] > 1){
                if((x > 20) && (x < (40)) && (y < height/3 ) && (y > (height/3 - 40))){
                   std::memset(&p_other_frame[offset], 0xC0, other_bpp); 
                }
            }
            if (intensities[0] > 2){
                if((x > (40)) && (x < (60)) && (y < height/3 ) && (y > (height/3 - 50))){
                   std::memset(&p_other_frame[offset], 0xD0, other_bpp); 
                }
            }
            if (intensities[0] > 3){
                if((x > (60)) && (x < (80)) && (y < height/3 ) && (y > (height/3 - 60))){
                   std::memset(&p_other_frame[offset], 0xE0, other_bpp); 
                }
            }
            if (intensities[0] > 4){
                if((x > (80)) && (x < (100)) && (y < height/3 ) && (y > (height/3 - 70))){
                   std::memset(&p_other_frame[offset], 0xFF, other_bpp); 
                }
            }
            
            // Feather 2
            if (intensities[1] > 0){
                if((x > (width/3)) && (x < ((width/3)+20)) && (y < height/3 ) && (y > (height/3 - 30))){
                   std::memset(&p_other_frame[offset], 0xB0, other_bpp); 
                }
            }
            if (intensities[1] > 1){
                if((x > (width/3)+20) && (x < (width/3 + 40)) && (y < height/3 ) && (y > (height/3 - 40))){
                   std::memset(&p_other_frame[offset], 0xC0, other_bpp); 
                }
            }
            if (intensities[1] > 2){
                if((x > (width/3 + 40)) && (x < (width/3 + 60)) && (y < height/3 ) && (y > (height/3 - 50))){
                   std::memset(&p_other_frame[offset], 0xD0, other_bpp); 
                }
            }
            if (intensities[1] > 3){
                if((x > (width/3 + 60)) && (x < (width/3 + 80)) && (y < height/3 ) && (y > (height/3 - 60))){
                   std::memset(&p_other_frame[offset], 0xE0, other_bpp); 
                }
            }
            if (intensities[1] > 4){
                if((x > (width/3 + 80)) && (x < (width/3 + 100)) && (y < height/3 ) && (y > (height/3 - 70))){
                   std::memset(&p_other_frame[offset], 0xFF, other_bpp); 
                }
            }
            
            // Feather 3
            if (intensities[2] > 0){
                if((x > (2*(width/3))) && (x < (2*(width/3)+20)) && (y < height/3 ) && (y > (height/3 - 30))){
                   std::memset(&p_other_frame[offset], 0xB0, other_bpp); 
                }
            }
            if (intensities[2] > 1){
                if((x > (2*(width/3))+20) && (x < (2*(width/3) + 40)) && (y < height/3 ) && (y > (height/3 - 40))){
                   std::memset(&p_other_frame[offset], 0xC0, other_bpp); 
                }
            }
            if (intensities[2] > 2){
                if((x > (2*(width/3) + 40)) && (x < (2*(width/3) + 60)) && (y < height/3 ) && (y > (height/3 - 50))){
                   std::memset(&p_other_frame[offset], 0xD0, other_bpp); 
                }
            }
            if (intensities[2] > 3){
                if((x > (2*(width/3) + 60)) && (x < (2*(width/3) + 80)) && (y < height/3 ) && (y > (height/3 - 60))){
                   std::memset(&p_other_frame[offset], 0xE0, other_bpp); 
                }
            }
            if (intensities[2] > 4){
                if((x > (2*(width/3) + 80)) && (x < (2*(width/3) + 100)) && (y < height/3 ) && (y > (height/3 - 70))){
                   std::memset(&p_other_frame[offset], 0xFF, other_bpp); 
                }
            }
            
            // Feather 4
            if (intensities[3] > 0){
                if((x > 0) && (x < (20)) && (y < 2*(height/3)) && (y > (2*(height/3) - 30))){
                   std::memset(&p_other_frame[offset], 0xB0, other_bpp); 
                }
            }
            if (intensities[3] > 1){
                if((x > 20) && (x < (40)) && (y < 2*(height/3)) && (y > (2*(height/3) - 40))){
                   std::memset(&p_other_frame[offset], 0xC0, other_bpp); 
                }
            }
            if (intensities[3] > 2){
                if((x > (40)) && (x < (60)) && (y < 2*(height/3)) && (y > (2*(height/3) - 50))){
                   std::memset(&p_other_frame[offset], 0xD0, other_bpp); 
                }
            }
            if (intensities[3] > 3){
                if((x > (60)) && (x < (80)) && (y < 2*(height/3)) && (y > (2*(height/3) - 60))){
                   std::memset(&p_other_frame[offset], 0xE0, other_bpp); 
                }
            }
            if (intensities[3] > 4){
                if((x > (80)) && (x < (100)) && (y < 2*(height/3)) && (y > (2*(height/3) - 70))){
                   std::memset(&p_other_frame[offset], 0xFF, other_bpp); 
                }
            }
            
            //Feather 5
            if (intensities[4] > 0){
                if((x > width/3) && (x < ((width/3)+20)) && (y < 2*(height/3)) && (y > (2*(height/3) - 30))){
                   std::memset(&p_other_frame[offset], 0xB0, other_bpp); 
                }
            }
            if (intensities[4] > 1){
                if((x > (width/3)+20) && (x < ((width/3)+40)) && (y < 2*(height/3)) && (y > (2*(height/3) - 40))){
                   std::memset(&p_other_frame[offset], 0xC0, other_bpp); 
                }
            }
            if (intensities[4] > 2){
                if((x > ((width/3)+40)) && (x < ((width/3)+60)) && (y < 2*(height/3)) && (y > (2*(height/3) - 50))){
                   std::memset(&p_other_frame[offset], 0xD0, other_bpp); 
                }
            }
            if (intensities[4] > 3){
                if((x > ((width/3)+60)) && (x < ((width/3)+80)) && (y < 2*(height/3)) && (y > (2*(height/3) - 60))){
                   std::memset(&p_other_frame[offset], 0xE0, other_bpp); 
                }
            }
            if (intensities[4] > 4){
                if((x > ((width/3)+80)) && (x < ((width/3)+100)) && (y < 2*(height/3)) && (y > (2*(height/3) - 70))){
                   std::memset(&p_other_frame[offset], 0xFF, other_bpp); 
                }
            }
            
            //Feather 6
            if (intensities[5] > 0){
                if((x > 2*(width/3)) && (x < (2*(width/3)+20)) && (y < 2*(height/3)) && (y > (2*(height/3) - 30))){
                   std::memset(&p_other_frame[offset], 0xB0, other_bpp); 
                }
            }
            if (intensities[5] > 1){
                if((x > 2*(width/3)+20) && (x < (2*(width/3)+40)) && (y < 2*(height/3)) && (y > (2*(height/3) - 40))){
                   std::memset(&p_other_frame[offset], 0xC0, other_bpp); 
                }
            }
            if (intensities[5] > 2){
                if((x > (2*(width/3)+40)) && (x < (2*(width/3)+60)) && (y < 2*(height/3)) && (y > (2*(height/3) - 50))){
                   std::memset(&p_other_frame[offset], 0xD0, other_bpp); 
                }
            }
            if (intensities[5] > 3){
                if((x > (2*(width/3)+60)) && (x < (2*(width/3)+80)) && (y < 2*(height/3)) && (y > (2*(height/3) - 60))){
                   std::memset(&p_other_frame[offset], 0xE0, other_bpp); 
                }
            }
            if (intensities[5] > 4){
                if((x > (2*(width/3)+80)) && (x < (2*(width/3)+100)) && (y < 2*(height/3)) && (y > (2*(height/3) - 70))){
                   std::memset(&p_other_frame[offset], 0xFF, other_bpp); 
                }
            }
            
            //Feather 7
            if (intensities[6] > 0){
                if((x > 0) && (x < (20)) && (y < 3*(height/3)) && (y > (3*(height/3) - 30))){
                   std::memset(&p_other_frame[offset], 0xB0, other_bpp); 
                }
            }
            if (intensities[6] > 1){
                if((x > 20) && (x < (40)) && (y < 3*(height/3)) && (y > (3*(height/3) - 40))){
                   std::memset(&p_other_frame[offset], 0xC0, other_bpp); 
                }
            }
            if (intensities[6] > 2){
                if((x > (40)) && (x < (60)) && (y < 3*(height/3)) && (y > (3*(height/3) - 50))){
                   std::memset(&p_other_frame[offset], 0xD0, other_bpp); 
                }
            }
            if (intensities[6] > 3){
                if((x > (60)) && (x < (80)) && (y < 3*(height/3)) && (y > (3*(height/3) - 60))){
                   std::memset(&p_other_frame[offset], 0xE0, other_bpp); 
                }
            }
            if (intensities[6] > 4){
                if((x > (80)) && (x < (100)) && (y < 3*(height/3)) && (y > (3*(height/3) - 70))){
                   std::memset(&p_other_frame[offset], 0xFF, other_bpp); 
                }
            }
            
            //Feather 8
            if (intensities[7] > 0){
                if((x > (width/2)) && (x < (width/2 + 20)) && (y < 3*(height/3)) && (y > (3*(height/3) - 30))){
                   std::memset(&p_other_frame[offset], 0xB0, other_bpp); 
                }
            }
            if (intensities[7] > 1){
                if((x > (width/2)+ 20) && (x < (width/2 + 40)) && (y < 3*(height/3)) && (y > (3*(height/3) - 40))){
                   std::memset(&p_other_frame[offset], 0xC0, other_bpp); 
                }
            }
            if (intensities[7] > 2){
                if((x > (width/2 + 40)) && (x < (width/2 + 60)) && (y < 3*(height/3)) && (y > (3*(height/3) - 50))){
                   std::memset(&p_other_frame[offset], 0xD0, other_bpp); 
                }
            }
            if (intensities[7] > 3){
                if((x > (width/2 + 60)) && (x < (width/2 + 80)) && (y < 3*(height/3)) && (y > (3*(height/3) - 60))){
                   std::memset(&p_other_frame[offset], 0xE0, other_bpp); 
                }
            }
            if (intensities[7] > 4){
                if((x > (width/2 + 80)) && (x < (width/2 + 100)) && (y < 3*(height/3)) && (y > (3*(height/3) - 70))){
                   std::memset(&p_other_frame[offset], 0xFF, other_bpp); 
                }
            }
        }
    }
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

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}
