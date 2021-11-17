#ifndef NSRA_COMPUTER_VISION__NSRA_BOTTLE_DETECTION_H
#define NSRA_COMPUTER_VISION__NSRA_BOTTLE_DETECTION_H

#include "nsra_odrive_interface/coords.h"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace nsra_bottle_detection
{
    class NSRABottleDetection
    {
        public:

        cv::Mat points4d;
        Mat cam0pnts(1,1,CV_64FC2);
        Mat cam1pnts(1,1,CV_64FC2);
        Mat P1, P2;

        ros::ServiceClient right_camera;
        ros::ServiceClient left_camera;
    }
}

#endif