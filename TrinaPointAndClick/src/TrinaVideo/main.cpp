#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>       /* sin */
#define CVUI_IMPLEMENTATION
#include "cvui.h"
#define WINDOW_NAME	"Autonomous Grasping"



int main(int argc, char *argv[])
{
     cv::Mat frame = cv::Mat(480, 640, CV_8UC3);
    cv::Mat image;
    cv::VideoCapture in_video;

    in_video.open(0);//Camera index should be a passed parameter

    
    cvui::init(WINDOW_NAME);



    while (in_video.grab()){ //Loop while video exists
    
        in_video.retrieve(image);
        
        cvui::image(frame, 0, 0, image);

       

        cvui::update();
        cv::imshow(WINDOW_NAME, frame);

        // Check if ESC key was pressed
            if (cv::waitKey(20) == 27|| cv::getWindowProperty(WINDOW_NAME, cv::WND_PROP_ASPECT_RATIO) < 0) {
            break;
        }
    }

    in_video.release();
    return 0;
}
