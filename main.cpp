/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images

   NOTE, IF YOU WANT TO SPEED UP THE REMAP FUNCTION I STRONGLY RECOMMEND TO INSTALL
   INTELL IPP LIBRARIES ( http://software.intel.com/en-us/intel-ipp/ )
   YOU JUST NEED TO INSTALL IT AND INCLUDE ipp.h IN YOUR PROGRAM

   Copyright (C) 2009 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "fisheye_param.h"
#include <iostream>

const int slider_max = 5000;
cv::Mat src, dst, dst_with_cylinder;
cv::Mat mapx_persp, mapy_persp;
cv::Mat mapx_with_cylinder, mapy_with_cylinder;
cv::Mat K;
int fx, fy;
int xc, yc;
FisheyeParam ocam_model;
int f_cylinder;
cv::Mat cylinder;
int cylinder_roi = 100;
float theta1 = 20*CV_PI/180, theta2 = -60*CV_PI/180;

void create_perspecive_undistortion_LUT(const cv::Mat K, cv::Mat mapx, cv::Mat mapy, FisheyeParam& ocam_model)
{
  cv::Size image_size = mapx.size();
  //std::cout << image_size.width << " " << image_size.height << std::endl;
  const float &fx = K.at<float>(0, 0), &fy = K.at<float>(1, 1);
  const float &xc = K.at<float>(0, 2), &yc = K.at<float>(1, 2);
  for(int i = 0; i < image_size.height; i++)
    for(int j = 0; j < image_size.width; j++)
    {
      cv::Point3f point3d;
      point3d.x = (j - xc) / fx;
      point3d.y = (i - yc) / fy;
      point3d.z = 1;
      cv::Point2f point2d = ocam_model.World2Camera(point3d);
      mapx.at<float>(i, j) = point2d.x;
      mapy.at<float>(i, j) = point2d.y;
    }
}

void Fisheye2twoPerspective(cv::Mat src, cv::Mat dst, const cv::Mat& K, FisheyeParam& ocam_model)
{
  cv::Size image_size = src.size();
  float theta1 = 70*CV_PI/180.0, theta2 = -10*CV_PI/180.0;
  //std::cout << image_size.width << " " << image_size.height << std::endl;
  const float &fx = K.at<float>(0, 0), &fy = K.at<float>(1, 1);
  const float &xc = K.at<float>(0, 2), &yc = K.at<float>(1, 2);
  for(int i = 0; i < image_size.height; i++)
    for(int j = 0; j < image_size.width; j++)
    {
      cv::Point2f point2d;
      point2d.x = j;
      point2d.y = i;
      cv::Point3f point3d = ocam_model.Camera2World(point2d);
      float x = point3d.x;
      float y = point3d.y;
      float z = point3d.z;
      int u, v;
      if(x <= -tan((theta1+theta2)/2)*point3d.z)
      {
        float alpha = (theta1-theta2)/2;
        point3d.x = z*sin(theta1) + x*cos(theta1);
        point3d.y = y; 
        point3d.z = z*cos(theta1) - x*sin(theta1);
        //u = point3d.x/point3d.z*fx + xc - fx*tan(theta1);
        u = point3d.x/point3d.z*fx + xc - fx*tan(alpha);
        v = point3d.y/point3d.z*fy + yc;
      }
      else
      {
        float alpha = -(theta1-theta2)/2;
        point3d.x = z*sin(theta2) + x*cos(theta2);
        point3d.y = y;
        point3d.z = z*cos(theta2) - x*sin(theta2);
        //u = point3d.x/point3d.z*fx + xc - fx*tan(theta2);
        u = point3d.x/point3d.z*fx + xc - fx*tan(alpha);
        v = point3d.y/point3d.z*fy + yc;
      }
      
      if(u >= 0 && u < dst.cols && v >= 0 && v < dst.rows)
        dst.at<cv::Vec3b>(v, u) = src.at<cv::Vec3b>(i,j);
    }
}


void Fisheye2twoPerspective2(cv::Mat src, cv::Mat dst, float fx1, float fy1, float fx2, float fy2, float theta1, float theta2, FisheyeParam& ocam_model)
{
  if(src.empty() || dst.empty())
  {
    std::cout << __FILE__ << ": "<< __LINE__ << " error!" << std::endl;
    return;
  }

  cv::Size image_size = dst.size();
  //std::cout << image_size.width << std::endl;
      
  float xc1 = xc - fx1 * tan((theta1-theta2)/2), yc1 = yc;
  float xc2 = xc - fx2 * tan(-(theta1-theta2)/2), yc2 = yc;
  //float xc1 = xc - fx1 * tan(theta1), yc1 = yc;
  //float xc2 = xc - fx2 * tan(theta2), yc2 = yc;
  
  for(int i = 0; i < image_size.height; i++)
    for(int j = 0; j < image_size.width; j++)
    {
      cv::Point3f point3d;
      float x, y, z;
      if(j <= xc )
      {
        x = (j-xc1) / fx1;
        y = (i-yc1) / fy1;
        z = 1;
        float theta = -theta1;
        point3d.x = z*sin(theta) + x*cos(theta);
        point3d.y = y;
        point3d.z = z*cos(theta) - x*sin(theta);
      }
      else
      {
        x = (j-xc2) / fx2;
        y = (i-yc2) / fy2;
        z = 1;
        float theta = -theta2;
        point3d.x = z*sin(theta) + x*cos(theta);
        point3d.y = y;
        point3d.z = z*cos(theta) - x*sin(theta);
      }
      cv::Point2f point2d = ocam_model.World2Camera(point3d);
      int u = point2d.x, v = point2d.y;
      if(u >= 0 && u < src.cols && v >= 0 && v < src.rows)
        dst.at<cv::Vec3b>(i,j) = src.at<cv::Vec3b>(v,u);
    }
}


void create_perspecive_undistortion_with_cylinder_LUT(const cv::Mat K, const int f, cv::Mat mapx, cv::Mat mapy, FisheyeParam& ocam_model)
{
  cv::Size image_size = mapx.size();
  const float &fx = K.at<float>(0, 0), &fy = K.at<float>(1, 1);
  const float &xc = K.at<float>(0, 2), &yc = K.at<float>(1, 2);
  for(int i = 0; i < image_size.height; i++)
    for(int j = 0; j < image_size.width; j++)
    {
      cv::Point3f point3d;
      point3d.x = (j - xc) / fx;
      point3d.y = (i - yc) / fy;
      point3d.z = 1;
      cv::Point2f point2d;
      point2d = ocam_model.World2Camera(point3d);
      if(j >= image_size.width/2 - cylinder_roi && j <= image_size.width/2 + cylinder_roi)
      {
        float x = point2d.x, y = point2d.y;
        point2d.x = f*atan((x-0.5*image_size.width)/f) + f*atan(0.5*image_size.width / f);
        point2d.y = f*(y-0.5*image_size.height)/sqrt(pow(x-0.5*image_size.height, 2)+f*f) + 0.5*image_size.height;
      }
      
      mapx.at<float>(i, j) = point2d.x;
      mapy.at<float>(i, j) = point2d.y;
    }
}

void Undistrotion2Cylinder(cv::Mat src, cv::Mat dst, float f)
{
  int rows = src.rows;
  int cols = src.cols;
  for(int i = 0; i < rows; i++)
    for(int j = 0; j < cols; j++)
    {
      int u = f*(i-0.5*rows)/sqrt(pow(j-0.5*cols, 2)+f*f) + 0.5*rows;
      int v = f*atan((j-0.5*cols)/f) + f*atan(0.5*cols / f);
      if(u >= 0 && u <= dst.rows && v >= 0 && v < dst.cols)
      {
        dst.at<cv::Vec3b>(u,v) = src.at<cv::Vec3b>(i,j);
      }
    }
}

void Onchange(int, void*)
{
  K = (cv::Mat_<float>(3, 3) << fx,  0, xc,
                                 0, fy, yc,
                                 0,  0, 1);
  create_perspecive_undistortion_LUT(K, mapx_persp, mapy_persp, ocam_model);
  cv::remap(src, dst_with_cylinder, mapx_persp, mapy_persp, cv::INTER_LINEAR);
  dst = cv::Mat(src.rows, src.cols, src.type(), cv::Scalar::all(0));
  //Fisheye2twoPerspective(src, dst, K, ocam_model);
  Fisheye2twoPerspective2(src, dst, fx, fy, fx, fy, theta1, theta2, ocam_model);
  cylinder = cv::Mat::zeros(dst.rows, 2*f_cylinder*atan(0.5*dst.cols/f_cylinder), dst.type());
  Undistrotion2Cylinder(dst, cylinder, f_cylinder);
  //cylinder = cv::Mat::zeros(src.rows, 2*f_cylinder*atan(0.5*src.cols/f_cylinder), src.type());
  //Undistrotion2Cylinder(src, cylinder, f_cylinder);
  //create_perspecive_undistortion_with_cylinder_LUT(K, f_cylinder, mapx_with_cylinder, mapy_with_cylinder, ocam_model);
  //cv::remap(src, dst_with_cylinder, mapx_with_cylinder, mapy_with_cylinder, cv::INTER_LINEAR);
  
  cv::imshow( "Undistorted Perspective Image", dst );
  cv::imshow( "Cylinder Image", cylinder );
  cv::imshow( "With Cylinder Image", dst_with_cylinder );
}

int main(int argc, char *argv[])
{
  /* --------------------------------------------------------------------*/
  /* Read the parameters of the omnidirectional camera from the TXT file */
  /* --------------------------------------------------------------------*/
  ocam_model.Load("../calib_results.txt");

  /* --------------------------------------------------------------------*/
  /* Allocate space for the unistorted images                            */
  /* --------------------------------------------------------------------*/
  src = cv::imread("../frame_vc10_105.bmp");      // source image 1
  dst = cv::Mat(src.rows, src.cols, src.type(), cv::Scalar::all(0));    // undistorted panoramic image
  dst_with_cylinder = cv::Mat(src.rows, src.cols, src.type(), cv::Scalar::all(0));
  //std::cout << src.type() << " " << CV_8UC3 << std::endl;

  mapx_persp = cv::Mat(src.rows, src.cols, CV_32FC1);
  mapy_persp = cv::Mat(src.rows, src.cols, CV_32FC1);
  //mapx_with_cylinder = cv::Mat(src.rows, src.cols, CV_32FC1);
  //mapy_with_cylinder = cv::Mat(src.rows, src.cols, CV_32FC1);

  /* --------------------------------------------------------------------  */
  /* Create Look-Up-Table for perspective undistortion                     */
  /* SF is kind of distance from the undistorted image to the camera       */
  /* (it is not meters, it is justa zoom fator)                            */
  /* Try to change SF to see how it affects the result                     */
  /* The undistortion is done on a  plane perpendicular to the camera axis */
  /* --------------------------------------------------------------------  */
  fx = 480, fy = 360, xc = dst.cols/2.0, yc = 650;//dst.rows/2.0;
  K = (cv::Mat_<float>(3, 3) << fx,  0, xc,
                                 0, fy, yc,
                                 0,  0, 1);
  //float sf = 500;
  //create_perspecive_undistortion_LUT(mapx_persp, mapy_persp, ocam_model, sf);
  create_perspecive_undistortion_LUT(K, mapx_persp, mapy_persp, ocam_model);
  cv::remap(src, dst_with_cylinder, mapx_persp, mapy_persp, cv::INTER_LINEAR);
  //Fisheye2twoPerspective(src, dst, K, ocam_model);
  Fisheye2twoPerspective2(src, dst, fx, fy, fx, fy, theta1, theta2, ocam_model);
  f_cylinder = 200;
  cylinder = cv::Mat::zeros(dst.rows, 2*f_cylinder*atan(0.5*dst.cols/f_cylinder), dst.type());
  Undistrotion2Cylinder(dst, cylinder, f_cylinder);
  //cylinder = cv::Mat::zeros(src.rows, 2*f_cylinder*atan(0.5*src.cols/f_cylinder), src.type());
  //Undistrotion2Cylinder(src, cylinder, f_cylinder);

  //create_perspecive_undistortion_with_cylinder_LUT(K, f_cylinder, mapx_with_cylinder, mapy_with_cylinder, ocam_model);
  //cv::remap(src, dst_with_cylinder, mapx_with_cylinder, mapy_with_cylinder, cv::INTER_LINEAR);
  
  cv::namedWindow( "Original fisheye camera image", 0 );
  cv::namedWindow( "Undistorted Perspective Image", 0 );
  cv::imshow( "Original fisheye camera image", src );
  cv::imshow( "Undistorted Perspective Image", dst );
  cv::namedWindow( "Cylinder Image", 0 );
  cv::imshow( "Cylinder Image", cylinder );
  cv::namedWindow( "With Cylinder Image", 0 );
  cv::imshow( "With Cylinder Image", dst_with_cylinder );


  cv::createTrackbar("fx", "Undistorted Perspective Image", &fx, slider_max, Onchange);
  cv::createTrackbar("fy", "Undistorted Perspective Image", &fy, slider_max, Onchange);
  cv::createTrackbar("xc", "Undistorted Perspective Image", &xc, slider_max, Onchange);
  cv::createTrackbar("yc", "Undistorted Perspective Image", &yc, slider_max, Onchange);
  cv::createTrackbar("f_cylinder", "Undistorted Perspective Image", &f_cylinder, slider_max, Onchange);
  cv::createTrackbar("cylinder_roi", "Undistorted Perspective Image", &cylinder_roi, dst_with_cylinder.cols/2, Onchange);

  /* --------------------------------------------------------------------*/
  /* Wait until key 'q' pressed                                              */
  /* --------------------------------------------------------------------*/
  while((char)cvWaitKey(10) != 'q');
 
  /* --------------------------------------------------------------------*/
  /* Save image                                                          */
  /* --------------------------------------------------------------------*/
  cv::imwrite("undistorted_perspective.jpg", dst);
  cv::imwrite("cylinder.jpg", cylinder);
  cv::imwrite( "With Cylinder Image.jpg", dst_with_cylinder );
  printf("\nImage %s saved\n","undistorted_perspective.jpg");
  printf("\nImage %s saved\n","cylinder.jpg");
  printf("\nImage %s saved\n","dst_with_cylinder.jpg");

  cv::destroyAllWindows();

  return 0;
}
