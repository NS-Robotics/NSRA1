#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "nsra_odrive_interface/lr_coords.h"
#include <ros/package.h>
#include <Processing.NDI.Lib.h>

using namespace std;
using namespace cv;

#define CAM_RIGHT "PC-NOA (TV0200110012)"
#define CAM_LEFT "PC-NOA (TV0200110013)"

cv::Mat points4d;
cv::Mat points4d1;
Mat cam_left_pnts(1,1,CV_64FC2);
Mat cam_right_pnts(1,1,CV_64FC2);
Mat cam_left_pnts1(1,1,CV_64FC2);
Mat cam_right_pnts1(1,1,CV_64FC2);
Mat PL, PR;

std::vector<Point3d> coordBuffer;

ros::ServiceClient cameras;

ros::Publisher pub;
ros::Publisher pub1;

cv::FileStorage fs1(ros::package::getPath("nsra_robot_vision") + "/" + "storedPoints.yml", cv::FileStorage::WRITE);

class Camera
{
    public:
        Camera(string cam_name)
        {
            // Create a finder
	        NDIlib_find_instance_t pNDI_find = NDIlib_find_create_v2();
	        //if (!pNDI_find) return 0;

            // Wait until there is one source
	        uint32_t no_sources = 0;
	        const NDIlib_source_t* p_sources = NULL;
	        while (no_sources != 2)
	        {	// Wait until the sources on the nwtork have changed
		        printf("Looking for sources ...\n");
		        NDIlib_find_wait_for_sources(pNDI_find, 1000/* One second */);
		        p_sources = NDIlib_find_get_current_sources(pNDI_find, &no_sources);
	        }

            pNDI_recv = NDIlib_recv_create_v3();
	        //if (!pNDI_recv) return 0;

            if(cam_name == p_sources[0].p_ndi_name)
            {
                NDIlib_recv_connect(pNDI_recv, p_sources + 0);
            } else if(cam_name == p_sources[1].p_ndi_name)
            {
                NDIlib_recv_connect(pNDI_recv, p_sources + 1);
            } else
            {
                cout << "Detected cameras don't match with the input names" << endl;
            }

            pNDI_framesync = NDIlib_framesync_create(pNDI_recv);
            
            NDIlib_find_destroy(pNDI_find);	
        }

        ~Camera()
        {
            NDIlib_recv_destroy(pNDI_recv);
        }

        void getFrame()
        {
            cout << "test1" << endl;
            NDIlib_video_frame_v2_t video_frame;
            cout << "test2" << endl;
		    NDIlib_framesync_capture_video(pNDI_framesync, &video_frame);
            cout << "test3" << endl;
            if (video_frame.p_data)
		    {
                cout << "test4" << endl;
                cv::Mat ret_frame(video_frame.yres, video_frame.xres, CV_8UC4);
                ret_frame.data = (uint8_t *)video_frame.p_data;
                cout << "test5" << endl;
                //cv::Mat out;
                //cvtColor(ret_frame, out, cv::COLOR_RGBA2BGR);
                printf("Works\n");
                imshow("Frame", ret_frame);
		    }
            NDIlib_framesync_free_video(pNDI_framesync, &video_frame);
            /*
            NDIlib_video_frame_v2_t video_frame;
            
            cv::Mat ret_black = Mat::zeros(2064, 3088, CV_8UC4);

            switch (NDIlib_recv_capture_v2(pNDI_recv, &video_frame, nullptr, nullptr, 5000))
		    {	// No data
			    case NDIlib_frame_type_none:
				    printf("No data received.\n");
                    break;
				    //return ret_black;

			    // Video data
			    case NDIlib_frame_type_video:
				    printf("Video data received (%dx%d).\n", video_frame.xres, video_frame.yres);
                    
                    try
                    {
                        cv::Mat ret_frame(video_frame.yres, video_frame.xres, CV_8UC4, ret_black);
                        memcpy(video_frame, ret_frame, video_frame.yres*video_frame.xres*4);
                        //uint8_t* data = video_frame.p_data;
                        Mat out;
                        cvtColor(ret_frame, out, cv::COLOR_RGBA2BGR);
                        printf("Works\n");
                        imshow("Frame", out);
                        NDIlib_recv_free_video_v2(pNDI_recv, &video_frame);
                        break;
                        //return out;
                    } catch (const std::exception& e)
                    {
                        printf("Error\n");
                        printf(e.what());
                        break;
                        //return ret_black;
                    }
                    break;
                    //ret_frame.data = (uint8_t*)video_frame.p_data;
                default:
                    printf("bruh\n");
                    //return ret_black;
                    break;
		    }
            */
        }

    private:
        NDIlib_recv_instance_t pNDI_recv;
        NDIlib_framesync_instance_t pNDI_framesync;
};

void saveCallback(const std_msgs::StringConstPtr& str)
{
    cv::Mat mat(coordBuffer, false);
    fs1 << "Points" << mat;
}

void calcCallback(const std_msgs::StringConstPtr& str)
{
    nsra_odrive_interface::lr_coords camera_coords;
    camera_coords.request.test = 1;
    cameras.call(camera_coords);
    cam_right_pnts.at<double>(0,0) = camera_coords.response.x_right;
    cam_right_pnts.at<double>(0,1) = camera_coords.response.y_right;
    cam_left_pnts.at<double>(0,0) = camera_coords.response.x_left;
    cam_left_pnts.at<double>(0,1) = camera_coords.response.y_left;
    cam_right_pnts1.at<double>(0,0) = camera_coords.response.x1_right;
    cam_right_pnts1.at<double>(0,1) = camera_coords.response.y1_right;
    cam_left_pnts1.at<double>(0,0) = camera_coords.response.x1_left;
    cam_left_pnts1.at<double>(0,1) = camera_coords.response.y1_left;

    triangulatePoints(PL,PR,cam_left_pnts,cam_right_pnts,points4d);
    triangulatePoints(PL,PR,cam_left_pnts1,cam_right_pnts1,points4d1);
    /*
    cv::Mat1f Thomogeneous(4, 1); 
    Thomogeneous(0) = pnts3D.at<double>(0,0);
    Thomogeneous(1) = pnts3D.at<double>(0,1);
    Thomogeneous(2) = pnts3D.at<double>(0,2);
    Thomogeneous(3) = pnts3D.at<double>(0,3);

    Mat Th = Thomogeneous.reshape(4);

    cv::Mat T;
    cv::convertPointsFromHomogeneous(Th, T);
    */

    std::vector<Point3d> results;

    Point3d point = Point3d(points4d.at<double>(0, 0) / points4d.at<double>(3, 0),
                            points4d.at<double>(1, 0) / points4d.at<double>(3, 0),
                            points4d.at<double>(2, 0) / points4d.at<double>(3, 0));
    results.emplace_back(point);

    std::vector<Point3d> results1;

    Point3d point1 = Point3d(points4d1.at<double>(0, 0) / points4d1.at<double>(3, 0),
                            points4d1.at<double>(1, 0) / points4d1.at<double>(3, 0),
                            points4d1.at<double>(2, 0) / points4d1.at<double>(3, 0));
    results1.emplace_back(point1);

    std_msgs::String msg;
    msg.data = to_string(results[0].x) + "/" + to_string(results[0].y) + "/" + to_string(results[0].z);
    pub.publish(msg);

    coordBuffer.push_back(results[0]);

    std::vector<Point3d> added_results;

    Point3d point3 = Point3d(abs(results[0].x - results1[0].x),
                             abs(results[0].y - results1[0].y),
                             abs(results[0].z - results1[0].z));
    
    added_results.emplace_back(point3);

    cout << "Point1 Results: " << results << endl;
    cout << "Point2 Results: " << results1 << endl;
    cout << "Added Results: " << added_results << endl;
    cout << "Distance: " << sqrt(added_results[0].x*added_results[0].x + added_results[0].y*added_results[0].y + added_results[0].z*added_results[0].z) << endl;

}

int main(int argc, char** argv)
{
    //if (!NDIlib_initialize()) return 0;

    //Camera cam_left(CAM_LEFT);
    //Camera cam_right(CAM_RIGHT);

    VideoCapture cap;

    cap.open(0, 0);

    cv::Mat frame;
    while(true)
    {
        //cam_left.getFrame();
        cap.read(frame);
        imshow("Live", frame);
        /*
        cout << frame.rows << endl;
        cout << frame.cols << endl;
        Mat out;
        cvtColor(frame, out, cv::COLOR_RGBA2RGB);
        if(out.rows != 0)
        {
            imshow("Frame", out);
        }
        */
        int key = waitKey(33);
        if(key == 27)
        {
            cout << "end" << endl;
            break;
        }

    }
    
    /*
    ros::init(argc, argv, "bottle_detection");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("calc3Dcoords", 1, calcCallback);
    ros::Subscriber sub1 = n.subscribe("saveCoords", 1, saveCallback);
    cameras = n.serviceClient<nsra_odrive_interface::lr_coords>("get2dcoords");
    pub = n.advertise<std_msgs::String>("PointCoords", 5);

    FileStorage fs(ros::package::getPath("nsra_robot_vision") + "/config/" + "cam_stereo.yml", FileStorage::READ);

    fs["PL"] >> PL;
    fs["PR"] >> PR;
    
    cout << PL << endl;
    cout << PR << endl;

    ros::spin();
    */

    NDIlib_destroy();

    return 0;

}