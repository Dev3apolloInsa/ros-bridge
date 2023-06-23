#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <algorithm>
#include "polyfit.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include "image_transport/image_transport.hpp"


using std::placeholders::_1;

//Global variable declaration
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
  
  void Linear_regression(std::vector<std::vector<double>> lines, std::vector<double> &line_x, std::vector<double> &line_y, double &a, double &b, bool &draw_line) {
        std::vector<double> coeffs;
        for (int i = 0; i < lines.size(); i++) {
            line_x.push_back(lines[i][0]);
            line_x.push_back(lines[i][2]);
            line_y.push_back(lines[i][1]);
            line_y.push_back(lines[i][3]);
        }
        if (line_x.size() > 0) {
            coeffs = polyfit(line_x, line_y, 1);   // y = a*x + b
            a = coeffs[1];
            b = coeffs[0];
            draw_line = true;
        }else {
            int m = 1;
            b = 1;
            draw_line = false;
        }
    }
    
    std::vector<std::vector<double>> TriLines(std::vector<std::vector<double>> lines, std::vector<float> interval, float threshold = 0.08){
        std::vector<std::vector<double>> Lines_tri;
        float somme = 0;
        int cmpt = 0;
        for(auto a : lines){
            if((a[4] <= interval[1]) && (a[4] >= interval[0])){
                somme = somme + a[4];
                cmpt = cmpt + 1;
            }
        }
        float moyenne = somme / cmpt;
        for(auto element : lines){
            if((element[4] <= moyenne + threshold) && (element[4] >= moyenne - threshold)){
                Lines_tri.push_back(element);
            }
        }
        return Lines_tri;
    }
    
  
  void draw_lines(cv::Mat img, cv::Mat img_src) {
    /*This function draws `lines` with `color` and `thickness`*/ 
    //The Hough transform algorithm extracts all the lines passing through each of our edge points and group them by similarity. The HoughLinesP function in OpenCV returns an array of lines organized by endpoints (x1, x1, x2, x2).
    double rho = 0.8;
    double theta = CV_PI/180;
    int threshold = 25;
    int min_line_len = 0;      // 50 
    int max_line_gap = 250;
    cv::HoughLinesP(img, lines, rho, theta, threshold, min_line_len, max_line_gap);
    line_img = cv::Mat::zeros(img.size(), img.type());
    
    //The Hough transformation returns several lines to us, using an equation we organize the lines according to their slope: The positive slopes are for the right lane and the negative slopes are for the left lane (right_lines/left_lines) ===> Sort lines
    std::vector<std::vector<double>> right_lines;
    std::vector<std::vector<double>> left_lines;
    for (size_t i=0; i<lines.size(); i++) {
        double x1 = lines[i][0];
        double y1 = lines[i][1];
        double x2 = lines[i][2];
        double y2 = lines[i][3];
        float a = (y2 - y1) / (x2 - x1);
        if (a >= 0) {
            right_lines.push_back({x1, y1, x2, y2, a});
        }
        else {
            left_lines.push_back({x1, y1, x2, y2, a});
        }
    }
    
    //Filter out lines with unacceptable slopes that throw off the intended slope of each line ===> Reject outliers
    std::vector<std::vector<double>> right_lines_tri = TriLines(right_lines, {0.6, 1.5});
    std::vector<std::vector<double>> left_lines_tri = TriLines(left_lines, {-1.1, -0.2});
    
    //Finally we merge the left and right sets using linear regression. Which is an attempt to find the best relationship between a group of points. ===> Linear regression
    std::vector<double> left_lines_X, left_lines_Y, right_lines_X, right_lines_Y, coeffs_left, coeffs_right;
    double left_coeffs_a, left_coeffs_b, right_coeffs_a, right_coeffs_b; 
    bool draw_line_left = false, draw_line_right = false;
    if (!right_lines_tri.empty() && !left_lines_tri.empty()) {
    
        Linear_regression(left_lines_tri, left_lines_X, left_lines_Y, left_coeffs_a, left_coeffs_b, draw_line_left);
        
        Linear_regression(right_lines_tri, right_lines_X, right_lines_Y, right_coeffs_a, right_coeffs_b, draw_line_right); 
    
        auto it = std::minmax_element(right_lines_Y.begin(), right_lines_Y.end());
        double y2_right = *it.first;   //min_y
        double y1_right = *it.second;  //max_y
    
        auto itt = std::minmax_element(left_lines_Y.begin(), left_lines_Y.end());
        double y2_left = *itt.first;   //min_y
        double y1_left = *itt.second;  //max_y
    
        double y1;
        double y2;
        if (y1_right > y1_left) {
            y1 = y1_right;
            y2 = y2_right;
        } else { 
            y1 = y1_left;
            y2 = y2_left;
        }
    
        double left_x1 = (y1 - left_coeffs_b)/left_coeffs_a;
        double left_x2 = (y2 - left_coeffs_b)/left_coeffs_a;
        double right_x1 = (y1 - right_coeffs_b)/right_coeffs_a;
        double right_x2 = (y2 - right_coeffs_b)/right_coeffs_a;   
    
        if (draw_line_left == 1)
            cv::line(img_src, cv::Point(left_x1, y1), cv::Point(left_x2, y2), cv::Scalar(0, 0, 255), 5, cv::LINE_AA);
    
        if (draw_line_right == 1)
            cv::line(img_src, cv::Point(right_x1, y1), cv::Point(right_x2, y2), cv::Scalar(0, 0, 255), 5, cv::LINE_AA);
    }else{
        return;
    }  
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
    
    cv::Mat img_clone = cv_ptr->image.clone();
    
    /* Call grayscale function ===> image_gray */
    grayscale(cv_ptr->image);
    
    /* Call gaussian function ===> image_blurred*/
    gaussian_blur(image_gray);
    
    /* Call canny function ===> image_canny */
    canny(image_blurred);
    
    /* Call region_of_interest function ===> image_ROI */
    region_of_interest(image_canny);
    
    /* Call the fonction that draw the region of interest */
    draw_ROI(img_clone);
    
    /* Call the fonction that draw the dectected lanes ===> cv_ptr->image*/
    draw_lines(image_ROI, cv_ptr->image);
    
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
