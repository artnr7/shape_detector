// #include <iostream>

#include "shape_detector.hpp"
#include <opencv2/core/hal/interface.h>

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>

void SetParams(cv::SimpleBlobDetector::Params &params);
void ImgRead(const char **argv, cv::Mat &img,
             const enum cv::ImreadModes &img_read_mode);

int main(int argc, char **argv) {
  (void)argc;
  // (void)argv;

  // cv::VideoCapture capture(-1);
  // cv::Mat frame;

  // for (;;) {
  //   capture >> frame;
  //   if (frame.empty()) break;
  //   imshow("w", frame);
  //   cv::waitKey(20);  // waits to display frame
  // }
  // cv::waitKey(0);  // key press to close window
  //------------------------------------------------------------------------------
  // cv::Mat img1;
  // ImgRead((const char **)argv, img1, cv::ImreadModes::IMREAD_COLOR);

  // cv::Mat img2;
  // cv::cvtColor(img1, img2, cv::COLOR_BGR2GRAY);
  // for (int i = 0; i < img2.rows; i++) {
  //   for (int j = 0; j < img2.cols; j++) {
  //     cv::Vec3b pixel =
  //         img2.at<cv::Vec3b>(i, j);  // Vec3b для 3-х канального uchar
  //     std::cout << "Pixel (" << i << "," << j << "): B=" << (int)pixel[0]
  //               << " G=" << (int)pixel[1] << " R=" << (int)pixel[2]
  //               << std::endl;
  //   }
  //   break;
  // }

  // cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR_BGR);
  // // cv::threshold(img2, img, 150, 255, cv::THRESH_BINARY);

  // auto detector = cv::SimpleBlobDetector::create();

  // cv::SimpleBlobDetector::Params params{};
  // SetParams(params);
  // detector->setParams(params);

  // std::vector<cv::KeyPoint> keypoints;
  // detector->detect(img, keypoints);

  // cv::Mat blob_img;
  // cv::drawKeypoints(img, keypoints, blob_img, cv::Scalar(0, 0, 255),
  //                   cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  // cv::Mat blur_img;
  // cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0);

  // cv::Mat sobel_img;
  // cv::Sobel(blur_img, sobel_img, CV_64F, 1, 1, 5);

  // cv::Mat edges;
  // cv::Canny(blur_img, edges, 100, 200, 3, false);
  // // imshow("Shape Detector", img);
  // // imshow("Shape Detector → Blob", blob_img);
  // // imshow("Shape Detector → Blur", blur_img);
  // // imshow("Shape Detector → Sobel", sobel_img);
  // imshow("Shape Detector → Edges", edges);

  // cv::waitKey(0);

  // cv::destroyAllWindows();

  // imwrite("ch_img_1.jpg", img);
  //-----------------------------------------------------------------------
  // std::string mainw{"mainwin"};

  // cv::namedWindow(mainw, cv::WINDOW_AUTOSIZE);

  // cv::Mat img = cv::imread(argv[1], cv::ImreadModes::IMREAD_GRAYSCALE);

  // cv::GaussianBlur(img, img, cv::Size(3, 3), 0);

  // cv::Mat threshold;
  // int curpos = 155;

  // cv::createTrackbar("threshold", mainw, &curpos, 255);

  // while (1) {
  //   cv::threshold(img, threshold, curpos, 255, cv::THRESH_BINARY);
  //   cv::imshow(mainw, threshold);

  //   char c = cv::waitKey(33);
  //   if (c == 27) {
  //     break;
  //   }
  // }
  // cv::destroyAllWindows();

  //---------------------------------------------------------------------------
  // std::string mainw{"mainwin"};

  // cv::namedWindow(mainw, cv::WINDOW_AUTOSIZE);

  // cv::Mat img = cv::imread(argv[1], cv::ImreadModes::IMREAD_COLOR);

  // std::vector<cv::Mat> img_chnls;

  // cv::split(img, img_chnls);

  // cv::namedWindow("b", cv::WINDOW_NORMAL);
  // cv::namedWindow("g", cv::WINDOW_NORMAL);
  // cv::namedWindow("r", cv::WINDOW_NORMAL);

  // cv::resizeWindow("b", 1280, 720);
  // cv::resizeWindow("g", 1280, 720);
  // cv::resizeWindow("r", 1280, 720);

  // cv::imshow("img", img);
  // cv::imshow("b", img_chnls[0]);
  // cv::imshow("g", img_chnls[1]);
  // cv::imshow("r", img_chnls[2]);

  // cv::waitKey(0);

  // cv::destroyAllWindows();

  // --------------------------------------------------------------------------------

  // cv::Mat img = cv::imread(argv[1], cv::ImreadModes::IMREAD_COLOR);

  // cv::GaussianBlur(img, img, cv::Size(3, 3), 0);
  // cv::Mat hsv;
  // cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

  // std::vector<cv::Mat> img_chnls;
  // cv::split(hsv, img_chnls);

  // int all_max = 255;
  // double min = 0, max = 0;

  // std::string hnwin{"hn"};
  // std::string snwin{"sn"};
  // std::string vnwin{"vn"};

  // cv::namedWindow(hnwin, cv::WINDOW_NORMAL);
  // cv::namedWindow(snwin, cv::WINDOW_NORMAL);
  // cv::namedWindow(vnwin, cv::WINDOW_NORMAL);

  // cv::resizeWindow(hnwin, 640, 360);
  // cv::resizeWindow(snwin, 640, 360);
  // cv::resizeWindow(vnwin, 640, 360);

  // cv::minMaxLoc(img_chnls[0], &min, &max);
  // std::cout << min << "  " << max << std::endl;
  // int hmin = min;
  // int hmax = max;
  // cv::createTrackbar("hmin", hnwin, &hmin, 179);
  // cv::createTrackbar("hmax", hnwin, &hmax, 179);

  // cv::minMaxLoc(img_chnls[1], &min, &max);
  // std::cout << min << "  " << max << std::endl;

  // int smin = min, smax = max;
  // cv::createTrackbar("smin", snwin, &smin, all_max);
  // cv::createTrackbar("smax", snwin, &smax, all_max);

  // cv::minMaxLoc(img_chnls[2], &min, &max);
  // std::cout << min << "  " << max << std::endl;

  // int vmin = min, vmax = max;
  // cv::createTrackbar("vmin", vnwin, &vmin, all_max);
  // cv::createTrackbar("vmax", vnwin, &vmax, all_max);

  // cv::Mat hn, sn, vn;

  // cv::namedWindow("andwin", cv::WINDOW_NORMAL);
  // cv::resizeWindow("andwin", 1280, 720);
  // cv::Mat andn;

  // std::vector<cv::Vec3f> storage;
  // while (1) {
  //   cv::inRange(img_chnls[0], hmin, hmax, hn);
  //   cv::inRange(img_chnls[1], smin, smax, sn);
  //   cv::inRange(img_chnls[2], vmin, vmax, vn);

  //   // cv::imshow("hsv", hsv);
  //   cv::imshow(hnwin, hn);
  //   cv::imshow(snwin, sn);
  //   cv::imshow(vnwin, vn);
    
  //   cv::bitwise_and(hn, sn, andn);
  //   cv::bitwise_and(andn, vn, andn);


  //   cv::HoughCircles(andn, storage, cv::HOUGH_GRADIENT, 10, 100);

  //   for (size_t i = 0; i < storage.size(); i++) {
  //     cv::Point center{(int)storage[i][0], (int)storage[i][1]};

  //     int rad = (int)storage[i][2];

  //     cv::circle(andn, center, rad, cv::Scalar(0,0,255), 2);
  //   }

  //   cv::imshow("andwin",andn);

  //   auto detector = cv::SimpleBlobDetector::create();

  //   cv::SimpleBlobDetector::Params params{};
  //   SetParams(params);
  //   detector->setParams(params);

    // cv::imshow("andwin1", andn);

    // cv::Mat gray;
    // cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    // cv::threshold(gray, gray, 100, 255, cv::THRESH_BINARY);
    // cv::Mat Kern = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,
    // 5)); for (int i = 0; i < 3; ++i) {
    //   //   cv::dilate(andn, andn, Kern);
    //   cv::morphologyEx(andn, andn, cv::MorphTypes::MORPH_OPEN, Kern);
    // }

    // std::vector<cv::KeyPoint> keypoints;
    // detector->detect(andn, keypoints);

    // cv::Mat andn_blob;
    // cv::drawKeypoints(andn, keypoints, andn_blob, cv::Scalar(0, 0, 255),
    //                   cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // cv::imshow("andwin", andn_blob);

    // cv::imshow("h", img_chnls[0]);
    // cv::imshow("s", img_chnls[1]);
    // cv::imshow("v", img_chnls[2]);

  //   if (cv::waitKey(150) == 27) {
  //     break;
  //   }
  // }
  // cv::destroyAllWindows();


    (void)argc;

  std::string main_win{"Rawd Sign Detector"};
  cv::namedWindow(main_win, cv::WINDOW_NORMAL);

  cv::Mat img;
  sd::ImgReadFirstFile((const char**)argv, img, cv::IMREAD_COLOR);
  cv::Mat blur_img;
  cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0);

  cv::Mat hsv_img;
  cv::cvtColor(blur_img, hsv_img, cv::COLOR_BGR2HSV);

  std::string h_win{"h channel"};
  std::string s_win{"s channel"};
  std::string v_win{"v channel"};

  cv::namedWindow(h_win, cv::WINDOW_NORMAL);
  cv::namedWindow(s_win, cv::WINDOW_NORMAL);
  cv::namedWindow(v_win, cv::WINDOW_NORMAL);

  sd::ResizeWindows(main_win, h_win, s_win, v_win);

  std::vector<cv::Mat> channels;
  cv::split(hsv_img, channels);

  int h_min = 0, h_max = 0;
  int s_min = 0, s_max = 0;
  int v_min = 0, v_max = 0;

  sd::CreateTrackbars(channels, h_win, s_win, v_win, h_min, h_max, s_min, s_max,
                      v_min, v_max);

  std::vector<cv::Mat> channels_ranged;
  for (size_t i = 0; i < channels.size(); ++i) {
    channels_ranged.push_back({});
  }

  std::string edges_win{"Canny edges"};
  cv::namedWindow(edges_win, cv::WINDOW_NORMAL);
  cv::resizeWindow(edges_win, MAIN_WIN_W, MAIN_WIN_H);

  std::string hsv_bin_edges_img_sum_win{"Edges and channels"};
  cv::namedWindow(hsv_bin_edges_img_sum_win, cv::WINDOW_NORMAL);
  cv::resizeWindow(hsv_bin_edges_img_sum_win, MAIN_WIN_W, MAIN_WIN_H);

  cv::Mat edges_img;
  cv::Canny(blur_img, edges_img, 100, 200, 3, false);

  cv::Mat hsv_bin_edges_img_sum;

  while (true) {
    sd::ChannelsInRange(channels, channels_ranged, h_min, h_max, s_min, s_max,
                        v_min, v_max);

    cv::Mat hsv_bin_sum;

    sd::ChannelsSum(hsv_bin_sum, channels_ranged);
    sd::ShowImages(main_win, h_win, s_win, v_win, hsv_bin_sum, channels_ranged);

    cv::imshow(edges_win, edges_img);
    cv::moveWindow(edges_win, MAIN_WIN_X + MAIN_WIN_W + BORDER_X, MAIN_WIN_Y);

    cv::bitwise_and(hsv_bin_sum, edges_img, hsv_bin_edges_img_sum);
    cv::imshow(hsv_bin_edges_img_sum_win, hsv_bin_edges_img_sum);
    cv::moveWindow(hsv_bin_edges_img_sum_win,
                   MAIN_WIN_X + 2 * MAIN_WIN_W + BORDER_X, MAIN_WIN_Y);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(edges_img, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++) {
      cv::drawContours(hsv_bin_edges_img_sum, contours, static_cast<int>(i),
                       cv::Scalar(255, 0, 0), 2, cv::LINE_8, hierarchy, 0);
    }
    sd::MoveWindows(main_win, h_win, s_win, v_win);

    if (cv::waitKey(PAUSE) == 27) {
      break;
    }
  }

  cv::destroyAllWindows();
  return 0;
}

void SetParams(cv::SimpleBlobDetector::Params &params) {
  params.minThreshold = 100;
  params.maxThreshold = 200;

  params.filterByArea = true;
  params.minArea = 15;
  // params.maxArea = 1500;

  params.filterByCircularity = true;
  params.minCircularity = 0.01;

  // params.filterByConvexity = true;
  // params.minConvexity = 0.01;

  params.filterByInertia = true;
  params.minInertiaRatio = 0.01;

  params.filterByColor = true;
  params.blobColor = 255;
}

void ImgRead(const char **argv, cv::Mat &img,
             const enum cv::ImreadModes &img_read_mode) {
  img = imread(argv[1], img_read_mode);
}