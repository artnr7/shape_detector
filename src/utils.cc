#include <opencv2/highgui.hpp>
#include <string>
#include <vector>

#include "shape_detector.hpp"

void sd::MakeWindows(const std::string &main_win, const std::string &h_win,
                     const std::string &s_win, const std::string &v_win,
                     const std::string &hsv_chnls_sum_win,
                     const std::string &edges_win,
                     const std::string &hsv_chnls_and_edges_sum_win) {
  cv::namedWindow(main_win, cv::WINDOW_NORMAL);

  cv::namedWindow(h_win, cv::WINDOW_NORMAL);
  cv::namedWindow(s_win, cv::WINDOW_NORMAL);
  cv::namedWindow(v_win, cv::WINDOW_NORMAL);

#ifdef HSV
  cv::namedWindow(hsv_chnls_sum_win, cv::WINDOW_NORMAL);
#endif

#ifdef EDGES
  cv::namedWindow(edges_win, cv::WINDOW_NORMAL);
#endif

#ifdef HSV_PLUS_EDGES
  cv::namedWindow(hsv_chnls_and_edges_sum_win, cv::WINDOW_NORMAL);
#endif

#ifndef HSV
  (void)hsv_chnls_sum_win;
#endif

#ifndef EDGES
  (void)edges_win;
#endif

#ifndef HSV_PLUS_EDGES
  (void)hsv_chnls_and_edges_sum_win;
#endif
}

void sd::ResizeWindows(const std::string &main_win, const std::string &h_win,
                       const std::string &s_win, const std::string &v_win,
                       const std::string &hsv_chnls_sum_win,
                       const std::string &edges_win,
                       const std::string &hsv_chnls_and_edges_sum_win) {
  cv::resizeWindow(main_win, MAIN_WIN_W, MAIN_WIN_H);

  cv::resizeWindow(h_win, H_WIN_W, H_WIN_H);
  cv::resizeWindow(s_win, S_WIN_W, S_WIN_H);
  cv::resizeWindow(v_win, V_WIN_W, V_WIN_H);

#ifdef HSV
  cv::resizeWindow(hsv_chnls_sum_win, MAIN_WIN_W, MAIN_WIN_H);
#endif

#ifdef EDGES

  cv::resizeWindow(edges_win, MAIN_WIN_W, MAIN_WIN_H);
#endif

#ifdef HSV_PLUS_EDGES
  cv::resizeWindow(hsv_chnls_and_edges_sum_win, MAIN_WIN_W, MAIN_WIN_H);
#endif

  // (void)main_win;
  // (void)h_win;
  // (void)s_win;
  // (void)v_win;
#ifndef HSV
  (void)hsv_chnls_sum_win;
#endif

#ifndef EDGES
  (void)edges_win;
#endif

#ifndef HSV_PLUS_EDGES
  (void)hsv_chnls_and_edges_sum_win;
#endif
}

// Какую матрицу изображения где показывать
void sd::ShowImages(const std::string &main_win,
                    const cv::Mat &sign_detect_res_img,
                    const std::string &h_win, const std::string &s_win,
                    const std::string &v_win,
                    const std::vector<cv::Mat> channels_ranged,
                    const std::string &hsv_chnls_sum_win,
                    const cv::Mat &hsv_chnls_sum_img,
                    const std::string &edges_win, const cv::Mat &edges_img,
                    const std::string &hsv_chnls_and_edges_sum_win,
                    const cv::Mat &hsv_chnls_and_edges_sum_img) {
  cv::imshow(main_win, sign_detect_res_img);

  cv::imshow(h_win, channels_ranged[0]);
  cv::imshow(s_win, channels_ranged[1]);
  cv::imshow(v_win, channels_ranged[2]);

#ifdef HSV
  cv::imshow(hsv_chnls_sum_win, hsv_chnls_sum_img);
#endif

#ifdef EDGES
  cv::imshow(edges_win, edges_img);
#endif

#ifdef HSV_PLUS_EDGES
  cv::imshow(hsv_chnls_and_edges_sum_win, hsv_chnls_and_edges_sum_img);
#endif

  // (void)main_win;
  // (void)sign_detect_res_img;
  // (void)h_win;
  // (void)s_win;
  // (void)v_win;
  // (void)channels_ranged;

#ifndef HSV
  (void)hsv_chnls_sum_win;
  (void)hsv_chnls_sum_img;
#endif

#ifndef EDGES
  (void)edges_win;
  (void)edges_img;
#endif

#ifndef HSV_PLUS_EDGES
  (void)hsv_chnls_and_edges_sum_win;
  (void)hsv_chnls_and_edges_sum_img;
#endif
}

void sd::MoveWindows(const std::string &main_win, const std::string &h_win,
                     const std::string &s_win, const std::string &v_win,
                     const std::string &hsv_chnls_sum_win,
                     const std::string &edges_win,
                     const std::string &hsv_chnls_and_edges_sum_win) {
  cv::moveWindow(main_win, MAIN_WIN_X, MAIN_WIN_Y);

  cv::moveWindow(h_win, H_WIN_X, H_WIN_Y);
  cv::moveWindow(s_win, S_WIN_X, S_WIN_Y);
  cv::moveWindow(v_win, V_WIN_X, V_WIN_Y);

// cv::moveWindow(hsv_chnls_sum_win, MAIN_WIN_X + 2 * MAIN_WIN_W + BORDER_X,
//  MAIN_WIN_Y);
#ifdef EDGES
  cv::moveWindow(edges_win, MAIN_WIN_X + 2 * MAIN_WIN_W + BORDER_X, MAIN_WIN_Y);
#endif
  // cv::moveWindow(hsv_chnls_and_edges_sum_win,
  //  MAIN_WIN_X + MAIN_WIN_W + BORDER_X, MAIN_WIN_Y);

  // (void)main_win;
  // (void)h_win;
  // (void)s_win;
  // (void)v_win;
  (void)hsv_chnls_sum_win;
#ifndef EDGES
  (void)edges_win;
#endif
  (void)hsv_chnls_and_edges_sum_win;
}

void sd::ChannelsInRange(const std::vector<cv::Mat> &channels,
                         std::vector<cv::Mat> &channels_ranged, int h_min,
                         int h_max, int s_min, int s_max, int v_min,
                         int v_max) {
  cv::inRange(channels[0], h_min, h_max, channels_ranged[0]);
  cv::inRange(channels[1], s_min, s_max, channels_ranged[1]);
  cv::inRange(channels[2], v_min, v_max, channels_ranged[2]);
}

void sd::ChannelsSum(cv::Mat &hsv_bin_sum,
                     const std::vector<cv::Mat> channels_ranged) {
  cv::bitwise_and(channels_ranged[0], channels_ranged[1], hsv_bin_sum);
  cv::bitwise_and(channels_ranged[2], hsv_bin_sum, hsv_bin_sum);
}

void sd::ImgReadFirstFile(const char **argv, cv::Mat &img,
                          const enum cv::ImreadModes &img_read_mode) {
  img = imread(argv[1], img_read_mode);
}

void sd::SetParams(cv::SimpleBlobDetector::Params &params) {
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
bool sd::MainCycleState() {
  bool state = true;

  if (cv::waitKey(PAUSE) == 27) {
    state = false;
  }

  return state;
}

void sd::CreateTrackbars(const std::vector<cv::Mat> &channels,
                         const std::string &h_win, const std::string &s_win,
                         const std::string &v_win, int &h_min, int &h_max,
                         int &s_min, int &s_max, int &v_min, int &v_max) {
  int h_maxv = 179;
  int sv_max = 255;
  double min = 0, max = 0;

  cv::minMaxLoc(channels[0], &min, &max);
  // h_min = min;
  // h_max = max;
  cv::createTrackbar("Hmin", h_win, &h_min, h_maxv);
  cv::createTrackbar("Hmax", h_win, &h_max, h_maxv);

  cv::minMaxLoc(channels[1], &min, &max);

  // s_min = min;
  // s_max = max;
  cv::createTrackbar("Smin", s_win, &s_min, sv_max);
  cv::createTrackbar("Smax", s_win, &s_max, sv_max);

  cv::minMaxLoc(channels[2], &min, &max);

  // v_min = min;
  // v_max = max;
  cv::createTrackbar("Vmin", v_win, &v_min, sv_max);
  cv::createTrackbar("Vmax", v_win, &v_max, sv_max);
}

void sd::ChangeTrackbarsValues(int &h_min, int &h_max, int &s_min, int &s_max,
                               int &v_min, int &v_max) {
  h_min = 165;
  h_max = 179;
  s_min = 90;
  s_max = 255;
  v_min = 65;
  v_max = 188;
}

void sd::FindAndDrawContours(const cv::Mat &find_src, const cv::Mat &img,
                             cv::Mat &img_copy) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::findContours(find_src, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  img.copyTo(img_copy);
  for (size_t i = 0; i < contours.size(); i++) {
    cv::drawContours(img_copy, contours, static_cast<int>(i),
                     cv::Scalar(0, 0, 255), 20, cv::LINE_8, hierarchy, 0);
  }
}

void sd::ChannelsRangedCreate(const std::vector<cv::Mat> &channels,
                              std::vector<cv::Mat> &channels_ranged) {
  for (size_t i = 0; i < channels.size(); ++i) {
    channels_ranged.push_back({});
  }
}