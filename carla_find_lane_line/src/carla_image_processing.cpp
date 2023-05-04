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

//Global varible declaration
cv::Mat image_gray;
cv::Mat image_blurred;
cv::Mat image_canny;
cv::Mat image_ROI;
std::vector<cv::Vec4i> lines;
cv::Mat line_img;

class Image_processing : public rclcpp::Node
{
public:
  Image_processing() : Node("image_processing")
  {
    //Open demo window that will show output image
    cv::namedWindow(OPENCV_WINDOW);

    //Subscribe to camera topic 
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/carla/ego_vehicle/rgb_front/image", 10, std::bind(&Image_processing::topic_callback, this, _1));
  }
  
  ~Image_processing()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  
private:
  const std::string OPENCV_WINDOW = "CARLA IMAGE processing";
  void grayscale(cv::Mat img) {
    /*Applies the Grayscale transform to return an image with only one color channel*/
    cv::cvtColor(img, image_gray, CV_BGR2GRAY);
    return;
  }
  
  void gaussian_blur(cv::Mat img) {
    /*Applies a Gaussian Noise kernel to clean/remove noise*/
    cv::GaussianBlur(img, image_blurred, cv::Size(1, 1), 0);
    return;
  }
  
  void canny(cv::Mat img) {
    /*Applies the Canny transform to detect edges*/
    cv::Canny(img, image_canny, 100, 200);
    return;
  }
  
  void region_of_interest(cv::Mat img) {
    /*Applies an image mask. Keep only the region of the image defined by the polygon formed of "vertices".
      The rest of the image is highlighted in black. `vertices` must be a numeric array of integer points.*/
    int xsize = img.cols;
    int ysize = img.rows;
    int dx1 = int(0.0725 * xsize);
    int dx2 = int(0.425 * xsize);
    int dy = int(0.6 * ysize);
    std::vector<cv::Point> vertices;
    vertices = { cv::Point(dx1, ysize), cv::Point(dx2, dy), cv::Point(xsize - dx2, dy), cv::Point(xsize - dx1, ysize) };
    std::vector<std::vector<cv::Point>> pts{vertices};

    //cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
    cv::Mat mask = cv::Mat::zeros(img.size(), img.type());
    cv::fillPoly(mask, pts, cv::Scalar(255,255,255));
    cv::bitwise_and(img, mask, image_ROI);  
    return;
  }
  
  void draw_ROI(cv::Mat img) {
    /*Draw the region of interest on the image*/
    int xsize = img.cols;
    int ysize = img.rows;
    int dx1 = int(0.0725 * xsize);
    int dx2 = int(0.425 * xsize);
    int dy = int(0.6 * ysize);
    
    cv::line(img, cv::Point(dx1, ysize), cv::Point(dx2, dy), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    cv::line(img, cv::Point(dx2, dy), cv::Point(xsize - dx2, dy), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    cv::line(img, cv::Point(xsize - dx2, dy), cv::Point(xsize - dx1, ysize), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    cv::line(img, cv::Point(xsize - dx1, ysize), cv::Point(dx1, ysize), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    return;
  }  
  
  void draw_lines(cv::Mat img, cv::Mat img_src) {
    /*This function draws `lines` with `color` and `thickness`*/ 
    int rho = 1;
    float theta = CV_PI/180;
    int threshold = 50;
    int min_line_len = 80;
    int max_line_gap = 150;
    
    //The Hough transform algorithm extracts all the lines passing through each of our edge points and group them by similarity. The HoughLinesP function in OpenCV returns an array of lines organized by endpoints (x1, x1, x2, x2).
    cv::HoughLinesP(img, lines, rho, theta, threshold, min_line_len, max_line_gap);
    //line_img = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
    line_img = cv::Mat::zeros(img.size(), img.type());
    /*for (size_t i=0; i<lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::line(img_src, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }*/
    
    //The Hough transformation returns several lines to us, using an equation we organize the lines according to their slope: The positive slopes are for the right lane and the negative slopes are for the left lane (right_lines/left_lines) ===> Sort lines
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
        line(img_src, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    for (size_t j=0; j<left_lines.size(); j++) {
        std::vector<int> ll = left_lines[j];
        line(img_src, cv::Point(ll[0], ll[1]), cv::Point(ll[2], ll[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }*/
   
    //Filter out lines with unacceptable slopes that throw off the intended slope of each line ===> Reject outliers
    std::vector<std::vector<int>> right_lines_tri;
    std::vector<std::vector<int>> left_lines_tri;
    std::vector<float> right_interval = {0.45, 1.1};
    std::vector<float> left_interval = {-1.1, -0.2};
    float threshold_ = 0.08;
    float sum = 0;
    float sum_ = 0;
    int cmpt = 0;
    int cmpt_ = 0;
    for (auto a : right_lines){
        if (a[4] <= right_interval[1] && a[4] >= right_interval[0]){
            sum = sum + a[4];
            cmpt = cmpt + 1;
        }
    }
    float average = sum / cmpt;
    for (auto element : right_lines){
        if (element[4] <= average + threshold_ && element[4] >= average - threshold_){
            right_lines_tri.push_back(element);
        }
    }
    for (auto b : left_lines){
        if (b[4] <= left_interval[1] && b[4] >= left_interval[0]){
            sum_ = sum_ + b[4];
            cmpt_ = cmpt_ + 1;
        }
    }
    float average_ = sum_ / cmpt_;
    for (auto element : left_lines){
        if (element[4] <= average_ + threshold_ && element[4] >= average_ - threshold_){
            left_lines_tri.push_back(element);
        }
    }  
    for (size_t j=0; j<right_lines_tri.size(); j++) {
        std::vector<int> l = right_lines_tri[j];
        cv::line(img_src, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    for (size_t j=0; j<left_lines_tri.size(); j++) {
        std::vector<int> ll = left_lines_tri[j];
        cv::line(img_src, cv::Point(ll[0], ll[1]), cv::Point(ll[2], ll[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    
    //Finally we merge the left and right sets using linear regression. Which is an attempt to find the best relationship between a group of points. ===> Linear regression
    /*std::vector<int> right_lines_X; 
    std::vector<int> right_lines_Y;
    //std::vector<int> left_lines_X; 
    //std::vector<int> left_lines_Y;
    int right_a; 
    int right_b;
    //int left_a; 
    //int left_b;
    int m;
    int c;
    //int k;
    //int d;
    bool draw_line_right;
    //bool draw_line_left;
    //std::vector<int> coeffs1;
    std::vector<int> coeffs;
     
    for (int k=0; k < right_lines_tri.size(); k++) {
        right_lines_X.push_back(right_lines_tri[k][0]);
        right_lines_X.push_back(right_lines_tri[k][2]);
        right_lines_Y.push_back(right_lines_tri[k][1]);
        right_lines_Y.push_back(right_lines_tri[k][3]);   
    }
    /*for (int j=0; j < left_lines_tri.size(); j++) {
        left_lines_X.push_back(left_lines_tri[j][0]);
        left_lines_X.push_back(left_lines_tri[j][2]);
        left_lines_Y.push_back(left_lines_tri[j][1]);
        left_lines_Y.push_back(left_lines_tri[j][3]);   
    }*/
    
    /*if (right_lines_X.size() > 0) {
        coeffs = polyfit(right_lines_X, right_lines_Y, 1);  // y = a*x + b
        right_a = coeffs[0];
        right_b = coeffs[1];
        std::cout << right_a << std::endl;
        std::cout << right_b << std::endl;
        draw_line_right = true;
    } else {
        m = 1;
        c = 1;
        draw_line_right = false;
    }  
    /*if (left_lines_X.size() > 0) {
        coeffs1 = polyfit(left_lines_X, left_lines_Y, 1);  // y = a*x + b
        left_a = coeffs1[0];
        left_b = coeffs1[1];
        std::cout << left_a << std::endl;
        std::cout << left_b << std::endl;
        draw_line_left = true;
    } else {
        m = 1;
        c = 1;
        draw_line_left = false;
    }*/
    
    /*auto it = std::minmax_element(right_lines_Y.begin(), right_lines_Y.end());
    int y2_right = *it.first;   //min_y
    int y1_right = *it.second;  //max_y
    /*auto itt = std::minmax_element(left_lines_Y.begin(), left_lines_Y.end());
    int y2_left = *itt.first;   //min_y
    int y1_left = *itt.second;  //max_y*/ 
    /*int y1;
    int y2;
    
    y1 = y1_right;
    y2 = y2_right;

    /*if (y1_right > y1_left) {
        y1 = y1_right;
        y2 = y2_right;
    } else { 
        y1 = y1_left;
        y2 = y2_left;
    }*/
    /*
    int right_x1 = (y1 - right_b)/right_a;
    int right_x2 = (y2 - right_b)/right_a;
    //int left_x1 = (y1 - left_b)/left_a;
    //int left_x2 = (y2 - left_b)/left_a;
    
    if (draw_line_right == 1) 
    { 
        cv::line(img_src, cv::Point(right_x1, y1), cv::Point(right_x2, y2), cv::Scalar(0, 0, 255), 10, cv::LINE_AA);
    }  
    /*if (draw_line_left == 1) 
    { 
        line(img_src, cv::Point(left_x1, y1), cv::Point(left_x2, y2), cv::Scalar(0, 0, 255), 10, cv::LINE_AA);
    }*/  
  }
  
  //The callback function that run image procezssing 
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  
    /*Use cv_bridge to convert ROS image message in OpenCV image*/
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
    
    /* Call grayscale function ===> image_gray */
    grayscale(cv_ptr->image);
    
    /* Call gaussian function ===> image_blurred*/
    gaussian_blur(image_gray);
    
    /* Call canny function ===> image_canny */
    canny(image_blurred);
    
    /* Call region_of_interest function ===> image_ROI */
    region_of_interest(image_canny);
    
    /* Call the fonction that draw the region of interest */
    //draw_ROI(cv_ptr->image);
    
    /* Call the fonction that draw the dectected lanes */
    draw_lines(image_ROI, cv_ptr->image);
    
    /*cv::Mat dst;
    dst = image_canny.clone();
    cv::addWeighted(cv_ptr->image, 0.7, line_img, 0.3, 0., dst);*/

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
  
  
  
  
