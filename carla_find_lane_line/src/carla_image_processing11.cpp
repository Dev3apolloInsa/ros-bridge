// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "polyfit.hpp"
#include <algorithm>


using std::placeholders::_1;
//using namespace cv;

class Image_processing : public rclcpp::Node
{
public:
  Image_processing() : Node("image_processing")
  {
  
    // Open demo window that will show output image
    cv::namedWindow(OPENCV_WINDOW);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/carla/ego_vehicle/rgb_front/image", 10, std::bind(&Image_processing::topic_callback, this, _1));
  }
  
  ~Image_processing()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }



private:

  const std::string OPENCV_WINDOW = "CARLA IMAGE processing";
  
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    
    /* convert a color image to a gray scale */
    cv::Mat grayMat;
    cv::cvtColor(cv_ptr->image, grayMat, CV_BGR2GRAY);
    
    //std::cout << "Width : " << cv_ptr->image.cols << std::endl;
    //std::cout << "Height: " << cv_ptr->image.rows << std::endl;
    
    /* Blur the image with 3x3 Gaussian kernel */
    cv::Mat image_blurred_with_3x3_kernel;
    cv::GaussianBlur(grayMat, image_blurred_with_3x3_kernel, cv::Size(3, 3), 0); 
    
    /* Blur the image with 1x1 Gaussian kernel */
    cv::Mat image_blurred_with_1x1_kernel;
    cv::GaussianBlur(grayMat, image_blurred_with_1x1_kernel, cv::Size(1, 1), 0);
    
    /* Canny Edge Detection */
    cv::Mat CannyMat;
    cv::Canny(image_blurred_with_1x1_kernel, CannyMat, 100, 200);
    
    /* Region of interest */
    // calculate vertices for region of interest
    int xsize = CannyMat.cols;
    int ysize = CannyMat.rows;
    int dx1 = int(0.0725 * xsize);
    int dx2 = int(0.425 * xsize);
    int dy = int(0.6 * ysize);
    std::vector<cv::Point> vertices;
    
    vertices = { cv::Point(dx1, ysize), cv::Point(dx2, dy), cv::Point(xsize - dx2, dy), cv::Point(xsize - dx1, ysize) };
    
    std::vector<std::vector<cv::Point>> pts{vertices};
    //std::cout << vertices << std::endl;

    /*Line drawn using 8 connected*/
    //cv::line(cv_ptr->image, cv::Point(dx1, ysize), cv::Point(dx2, dy), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    
    //cv::line(cv_ptr->image, cv::Point(dx2, dy), cv::Point(xsize - dx2, dy), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    
    //cv::line(cv_ptr->image, cv::Point(xsize - dx2, dy), cv::Point(xsize - dx1, ysize), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    
    //cv::line(cv_ptr->image, cv::Point(xsize - dx1, ysize), cv::Point(dx1, ysize), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    
    /*defining a blank mask to start with*/
    cv::Mat mask = cv::Mat::zeros(CannyMat.rows, CannyMat.cols, CV_8UC1);
    
    cv::fillPoly(mask, pts, cv::Scalar(255,255,255));
    
    cv::Mat masked_image;
    
    cv::bitwise_and(CannyMat, mask, masked_image);  
    
    /*Draw lines*/
    /* void cv::HoughLines (InputArray image, OutputArray lines, double rho, double theta, int threshold,
                            double 	srn = 0, double stn = 0, double min_theta = 0, double max_theta = CV_PI)	
    
    ==> image       8-bit, single-channel binary source image
    ==> lines	    Output vector of lines
    ==> rho	        Distance resolution of the accumulator in pixels
    ==> theta	    Angle resolution of the accumulator in radians
    ==> threshold	Accumulator threshold paramete
    ==> srn	        For the multi-scale Hough transform, it is a divisor for the distance resolution rho
    ==> stn	        For the multi-scale Hough transform, it is a divisor for the distance resolution theta
    ==> min_theta   minimum angle to check for lines
    ==> max_theta   For standard and multi-scale Hough transform, an upper bound for the angle
    */
    std::vector<cv::Vec4i> lines;
    int rho = 1;
    float theta = CV_PI/180;
    int threshold = 50;
    int min_line_len = 80;
    int max_line_gap = 150;
    cv::HoughLinesP(masked_image, lines, rho, theta, threshold, min_line_len, max_line_gap);
    
    /*for (size_t i=0; i<lines.size(); i++) {
    cv::Vec4i l = lines[i];
    line(cv_ptr->image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }*/ 

    /*Separate lines (right_lines and left_lines*/
    std::vector<std::vector<int>> right_lines;
    std::vector<std::vector<int>> left_lines;
    
    for (size_t i=0; i<lines.size(); i++) {
        int x1 = lines[i][0];
        int y1 = lines[i][1];
        int x2 = lines[i][2];
        int y2 = lines[i][3];
        float a = (y2 - y1) / (x2 - x1);

        if (a >= 0) {
            right_lines.push_back({x1, y1, x2, y2, a});
        }
        else {
            left_lines.push_back({x1, y1, x2, y2, a});
        }
     }
    
    /*for (size_t j=0; j<right_lines.size(); j++) {
    std::vector<int> l = right_lines[j];
    line(cv_ptr->image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }*/
    
    /*Reject outliers*/
    std::vector<float> interval_right = {0.6, 1.5};
    std::vector<float> interval_left = {-1.1, -0.2};
    float threshold1 = 0.08;
    std::vector<std::vector<int>> Lines_tri_right;
    std::vector<std::vector<int>> Lines_tri_left;
    float somme1 = 0;
    int cmpt1 = 0;
    float somme2 = 0;
    int cmpt2 = 0;
    //right
    for (auto a : right_lines){
        if (a[4] <= interval_right[1] && a[4] >= interval_right[0]){
            somme1 = somme1 + a[4];
            cmpt1 = cmpt1 + 1;
        }
    }
    float moyenne1 = somme1 / cmpt1;
    for (auto element : right_lines){
        if (element[4] <= moyenne1 + threshold1 && element[4] >= moyenne1 - threshold1){
            Lines_tri_right.push_back(element);
        }
    }
    //left
    for (auto b : left_lines){
        if (b[4] <= interval_left[1] && b[4] >= interval_left[0]){
            somme2 = somme2 + b[4];
            cmpt2 = cmpt2 + 1;
        }
    }
    float moyenne2 = somme2 / cmpt2;
    for (auto element : left_lines){
        if (element[4] <= moyenne2 + threshold1 && element[4] >= moyenne2 - threshold1){
            Lines_tri_left.push_back(element);
        }
    }    
    
    /*for (size_t j=0; j<Lines_tri_right.size(); j++) {
    std::vector<int> l = Lines_tri_right[j];
    line(cv_ptr->image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }

    for (size_t j=0; j<Lines_tri_left.size(); j++) {
    std::vector<int> ll = Lines_tri_left[j];
    line(cv_ptr->image, cv::Point(ll[0], ll[1]), cv::Point(ll[2], ll[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }*/
    
    
    /*Linear regression:*/
    std::vector<int> right_lines_X; 
    std::vector<int> right_lines_Y;
    std::vector<double> v = {0, 1, 2, 3, 4};
	std::vector<double> w = {1, 1.8, 1.3, 2.5, 6.3};
    double right_a; 
    double right_b;
    double right_c;
    int m;
    bool draw_line_right;
    std::vector<double> coeffs;
     
    for (int k=0; k < Lines_tri_right.size(); k++) {
        right_lines_X.push_back(Lines_tri_right[k][0]);
        right_lines_X.push_back(Lines_tri_right[k][2]);
        right_lines_Y.push_back(Lines_tri_right[k][1]);
        right_lines_Y.push_back(Lines_tri_right[k][3]);   
    }
    if (right_lines_X.size() > 0) {
        //coeffs = polyfit(right_lines_X, right_lines_Y, 1);  // y = a*x + b
        coeffs = polyfit(v, w, 2);
        right_a = coeffs[0];
        right_b = coeffs[1];
        right_c = coeffs[2];
        std::cout << right_a << std::endl;
        std::cout << right_b << std::endl;
        std::cout << right_c << std::endl;
        draw_line_right = true;
    } else {
        m = 1;
        right_b = 1;
        draw_line_right = false;
    }
    
    /*std::vector<double> left_lines_X; 
    std::vector<double> left_lines_Y;
    double left_a; 
    double left_b;
    bool draw_line_left;
    std::vector<double> coeffs1;
    
    for (int k=0; k < Lines_tri_left.size(); k++) {
        left_lines_X.push_back(Lines_tri_left[k][0]);
        left_lines_X.push_back(Lines_tri_left[k][2]);
        left_lines_Y.push_back(Lines_tri_left[k][1]);
        left_lines_Y.push_back(Lines_tri_left[k][3]);
    }
    if (left_lines_X.size() > 0) {
        coeffs1 = polyfit(left_lines_X, left_lines_Y, 1);  // y = a*x + b
        left_a = coeffs1[0];
        left_b = coeffs1[1];
        draw_line_left = true;
    } else {
        left_a = 1;
        left_b = 1;
        draw_line_left = false;
    }*/
    
    auto it = std::minmax_element(right_lines_Y.begin(), right_lines_Y.end());
    //auto lt = std::minmax_element(left_lines_Y.begin(), left_lines_Y.end());
    int y2_right = *it.first;
    int y1_right = *it.second;
    //int y2_left = *lt.first;
    //int y1_left = *lt.second;
    
    int y1 = y1_right;
    int y2 = y2_right;
    /*if (y1_right > y1_left) {
        y1 = y1_right;
        y2 = y2_right;
    }
    else {
        y1 = y1_left;
        y2 = y2_left;
    }*/

    int right_x1 = (y1 - right_b) / right_a;
    int right_x2 = (y2 - right_b) / right_a;
    //int left_x1 = (y1 - left_b) / left_a;
    //int left_x2 = (y2 - left_b) / left_a;
 
    /*if (draw_line_right == 1) 
    { 
        line(cv_ptr->image, cv::Point(right_x1, y1), cv::Point(right_x2, y2), cv::Scalar(0, 0, 255), 10, cv::LINE_AA);
        std::cout << cv::Point(right_x1, y1) << std::endl;
        std::cout << cv::Point(right_x2, y2) << std::endl;
    }
    
    /*if (draw_line_left == 1 ) 
    { 
        line(cv_ptr->image, cv::Point(left_x1, y1), cv::Point(left_x2, y2), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }*/
    
    //cv::line(cv_ptr->image, cv::Point(-2, 555), cv::Point(-1, 365), cv::Scalar(0, 255, 0), 10);


    /*Show Carla vehicle camera image*/
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    RCLCPP_INFO(this->get_logger(), "Image published");
    
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Image_processing>());
  rclcpp::shutdown();
  return 0;
}
