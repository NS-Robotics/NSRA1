#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace std;
using namespace cv;

int x = 0;
Mat img1, img_res1;
char* imgs_directory;
char* extension;
char* name;

void imgSaveCallback(const std_msgs::String::ConstPtr& msg) {
  x++;
  char filename1[200];
  sprintf(filename1, "%s%s%d.%s", imgs_directory, name, x, extension);
  cout << "Saving " << name << " image " << x << endl;
  imwrite(filename1, img1);
}

int main(int argc, char** argv)
{
  char* cam_address;
  int im_width, im_height;

  static struct poptOption options[] = {
    { "img_width",'w',POPT_ARG_INT,&im_width,0,"Image width","NUM" },
    { "img_height",'h',POPT_ARG_INT,&im_height,0,"Image height","NUM" },
    { "imgs_directory",'d',POPT_ARG_STRING,&imgs_directory,0,"Directory to save images in","STR" },
    { "extension",'e',POPT_ARG_STRING,&extension,0,"Image extension","STR" },
    { "cam_address",'c',POPT_ARG_STRING,&cam_address,0,"RTSP Camera Address","STR" },
    { "name",'n',POPT_ARG_STRING,&name,0,"Camera Name","STR" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}

  const char *const_name = name;
  ros::init(argc, argv, const_name);
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("save_img", 1000, imgSaveCallback);

  VideoCapture cap1(cam_address);

  cap1.set(CAP_PROP_BUFFERSIZE, 2);
  //ros::Rate r(100);

  while (1) {
    cap1 >> img1;
    resize(img1, img_res1, Size(im_width, im_height));
    imshow(name, img_res1);
    int key = waitKey(30);
    if ((key != 255) && (key != 27)) {
      x++;
      char filename1[200];
      sprintf(filename1, "%s%s%d.%s", imgs_directory, name, x, extension);
      cout << "Saving " << name << " image " << x << endl;
      imwrite(filename1, img_res1);
    } else if(key == 27) {
      cout << "end" << endl;
      break;
    }
    ros::spinOnce();
    //r.sleep();
  }
  return 0;
}