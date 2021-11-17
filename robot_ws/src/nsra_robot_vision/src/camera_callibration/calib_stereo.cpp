#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"
#include <ros/package.h>

using namespace std;
using namespace cv;

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > imagePoints1, imagePoints2;
vector< Point2f > corners1, corners2;
vector< vector< Point2f > > left_img_points, right_img_points;

Mat img1, img2, gray1, gray2;

void load_image_points(int board_width, int board_height, int num_imgs, float square_size,
                      const char* leftimg_dir, const char* rightimg_dir, char* leftimg_filename, char* rightimg_filename, char* extension) {

  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;

  for (int i = 1; i <= num_imgs; i++) {
    char left_img[100], right_img[100], left_img_corners[100], right_img_corners[100];
    sprintf(left_img, "%s%s%d.%s", leftimg_dir, leftimg_filename, i, extension);
    sprintf(right_img, "%s%s%d.%s", rightimg_dir, rightimg_filename, i, extension);
    sprintf(left_img_corners, "%s%s%d_corners.%s", leftimg_dir, leftimg_filename, i, extension);
    sprintf(right_img_corners, "%s%s%d_corners.%s", rightimg_dir, rightimg_filename, i, extension);

    img1 = imread(left_img, cv::IMREAD_COLOR);
    img2 = imread(right_img, cv::IMREAD_COLOR);

    if(img1.empty() || img2.empty())
    {
      continue;
    }

    cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
    cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);

    bool found1 = false, found2 = false;

    found1 = cv::findChessboardCorners(img1, board_size, corners1,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    found2 = cv::findChessboardCorners(img2, board_size, corners2,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

    if(!found1 || !found2){
      cout << "Chessboard find error!" << endl;
      cout << "leftImg: " << left_img << " and rightImg: " << right_img <<endl;
      continue;
    } 

    if (found1)
    {
      cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray1, board_size, corners1, found1);
      cv::drawChessboardCorners(img1, board_size, corners1, found1);
      cv::imwrite(left_img_corners, img1);
    }
    if (found2)
    {
      cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray2, board_size, corners2, found2);
      cv::drawChessboardCorners(img2, board_size, corners2, found2);
      cv::imwrite(right_img_corners, img2);
    }

    vector< Point3f > obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

    if (found1 && found2) {
      cout << i << ". Found corners!" << endl;
      imagePoints1.push_back(corners1);
      imagePoints2.push_back(corners2);
      object_points.push_back(obj);
    }
  }
  
  for (int i = 0; i < imagePoints1.size(); i++) {
    vector< Point2f > v1, v2;
    for (int j = 0; j < imagePoints1[i].size(); j++) {
      v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
      v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
    }
    left_img_points.push_back(v1);
    right_img_points.push_back(v2);
  }
}

int main(int argc, char const *argv[])
{
  char* leftcalib_file;
  char* rightcalib_file;
  char* leftimg_dir;
  char* rightimg_dir;
  char* leftimg_filename;
  char* rightimg_filename;
  char* extension;
  char* out_file;
  int num_imgs;
  int square_size;

  static struct poptOption options[] = {
    { "num_imgs",'n',POPT_ARG_INT,&num_imgs,0,"Number of checkerboard images","NUM" },
    { "leftcalib_file",'u',POPT_ARG_STRING,&leftcalib_file,0,"Left camera calibration","STR" },
    { "rightcalib_file",'v',POPT_ARG_STRING,&rightcalib_file,0,"Right camera calibration","STR" },
    { "leftimg_filename",'l',POPT_ARG_STRING,&leftimg_filename,0,"Left image prefix","STR" },
    { "rightimg_filename",'r',POPT_ARG_STRING,&rightimg_filename,0,"Right image prefix","STR" },
    { "extension",'e',POPT_ARG_STRING,&extension,0,"Image extension","STR" },
    { "out_file",'o',POPT_ARG_STRING,&out_file,0,"Output calibration filename (YML)","STR" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}
  
  string img_dir = ros::package::getPath("nsra_robot_vision") + "/images/";
  string calib_dir_left = ros::package::getPath("nsra_robot_vision") + "/config/" + leftcalib_file;
  string calib_dir_right = ros::package::getPath("nsra_robot_vision") + "/config/" + rightcalib_file;

  cout << calib_dir_right << endl;
  cout << calib_dir_left << endl;

  FileStorage fsl(calib_dir_left.c_str(), FileStorage::READ);
  FileStorage fsr(calib_dir_right.c_str(), FileStorage::READ);

  fsl["square_size"] >> square_size;

  load_image_points(9, 6, num_imgs, square_size,
                   img_dir.c_str(), img_dir.c_str(), leftimg_filename, rightimg_filename, extension);

  printf("Starting Calibration\n");
  Mat KL, KR, R, F, E;
  Vec3d T;
  Mat DL, DR;
  fsl["camera_matrix"] >> KL;
  fsr["camera_matrix"] >> KR;
  fsl["distortion_coefficients"] >> DL;
  fsr["distortion_coefficients"] >> DR;
  int flag = 0;
  flag |= cv::CALIB_FIX_INTRINSIC;

  cout << KL << endl;
  cout << KR << endl;
  
  cout << "Read intrinsics" << endl;
  
  double rms = stereoCalibrate(object_points, left_img_points, right_img_points, KL, DL, KR, DR, img1.size(), R, T, E, F);

  cout << "Re-projection error reported by stereoCalibrate: "<< rms << endl;

  cv::FileStorage fs1(ros::package::getPath("nsra_robot_vision") + "/config/" + out_file, cv::FileStorage::WRITE);
  fs1 << "KL" << KL;
  fs1 << "KR" << KR;
  fs1 << "DL" << DL;
  fs1 << "DR" << DR;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;
  
  printf("Done Calibration\n");

  printf("Starting Rectification\n");

  cv::Mat RL, RR, PL, PR, Q;
  stereoRectify(KL, DL, KR, DR, img1.size(), R, T, RL, RR, PL, PR, Q);

  fs1 << "RL" << RL;
  fs1 << "RR" << RR;
  fs1 << "PL" << PL;
  fs1 << "PR" << PR;
  fs1 << "Q" << Q;

  printf("Done Rectification\n");

  return 0;
}
