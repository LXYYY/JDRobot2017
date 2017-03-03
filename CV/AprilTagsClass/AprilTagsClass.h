//
// Created by root on 2/26/17.
//

#ifndef JDROBOT_APRILTAGS_H
#define JDROBOT_APRILTAGS_H

#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <unistd.h>
#include <cmath>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

class AprilTagsClass {
private:
    AprilTags::TagDetector *m_tagDetector;
    AprilTags::TagCodes m_tagCodes;

    bool m_draw; // draw image and April tag detections?
    bool m_arduino; // send tag detections to serial port?
    bool m_timing; // print timing information for each tag extraction call

    int m_width; // image size in pixels
    int m_height;
    double m_tagSize; // April tag side length in meters of square black frame
    double m_fx; // camera focal length in pixels
    double m_fy;
    double m_px; // camera principal point
    double m_py;
public:
    vector<AprilTags::TagDetection> tagsL;
    vector<AprilTags::TagDetection> tagsR;

    AprilTagsClass();

    virtual ~AprilTagsClass();

    void setTagCodes(string s);

    bool setup();

    bool processImage(Mat image,vector<AprilTags::TagDetection>& detections);

    void drawTags(Mat& image,vector<AprilTags::TagDetection> tags);
};


#endif //JDROBOT_APRILTAGS_H
