//
// Created by kbisland on 11/5/19.
//

#include "AuxCameraDisplay.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "cvui.h"

static const cv::String& name = "Aux Camera";
static cv::Mat frame = cv::Mat(480, 640, CV_8UC3);
static std::string Cameras[4] = {
      //populate this array with the ros topics for the camera
};
void StartCameraDisplay(void){
    // Create a frame for this window and fill it with a nice color

    frame = cv::Scalar(49, 52, 49);
    cvui::init(name);
    cvui::update(name);
    //subscribe to other cameras in the list
    if (cvui::button(frame, 350, 420,120,40 , "&Previous")) {
        printf("PreviousCam");
    }

    if (cvui::button(frame, 200, 420,120,40 ,  "&Next")) {
        printf("NextCam");

    }

    // Show the content of this window on the screen
    cvui::imshow(name, frame);
}

cv::Mat convertImage(){

}


void UpdateDisplay(){

}

//void Imagecallback(const boost::shared_ptr<Message const>&){
//
//}

void monitorAuxCam(){

    if (cvui::button(frame, 350, 420,120,40, "&Previous")) {
        printf("PreviousCam");
    }

    if (cvui::button(frame, 200, 420,120,40,  "&Next")) {
        printf("NextCam");

    }

    cvui::update(name);
    cvui::imshow(name, frame);


}
