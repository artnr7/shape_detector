#include <vector>

#include "shape_detector.hpp"

void sd::ResizeWindows(const std::string &main_win, const std::string &h_win,
                       const std::string &s_win, const std::string &v_win) {
  cv::resizeWindow(main_win, MAIN_WIN_W, MAIN_WIN_H);

  cv::resizeWindow(h_win, H_WIN_W, H_WIN_H);
  cv::resizeWindow(s_win, S_WIN_W, S_WIN_H);
  cv::resizeWindow(v_win, V_WIN_W, V_WIN_H);
}

void sd::MoveWindows(const std::string &main_win, const std::string &h_win,
                     const std::string &s_win, const std::string &v_win) {
  cv::moveWindow(main_win, MAIN_WIN_X, MAIN_WIN_Y);

  cv::moveWindow(h_win, H_WIN_X, H_WIN_Y);
  cv::moveWindow(s_win, S_WIN_X, S_WIN_Y);
  cv::moveWindow(v_win, V_WIN_X, V_WIN_Y);
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
  h_min = min;
  h_max = max;
  cv::createTrackbar("Hmin", h_win, &h_min, h_maxv);
  cv::createTrackbar("Hmax", h_win, &h_max, h_maxv);

  cv::minMaxLoc(channels[1], &min, &max);

  s_min = min;
  s_max = max;
  cv::createTrackbar("Smin", s_win, &s_min, sv_max);
  cv::createTrackbar("Smax", s_win, &s_max, sv_max);

  cv::minMaxLoc(channels[2], &min, &max);

  v_min = min;
  v_max = max;
  cv::createTrackbar("Vmin", v_win, &v_min, sv_max);
  cv::createTrackbar("Vmax", v_win, &v_max, sv_max);
}

void sd::ShowImages(const std::string &main_win, const std::string &h_win,
                    const std::string &s_win, const std::string &v_win,
                    cv::Mat &hsv_img, const std::vector<cv::Mat> &channels) {
  cv::imshow(main_win, hsv_img);

  cv::imshow(h_win, channels[0]);
  cv::imshow(s_win, channels[1]);
  cv::imshow(v_win, channels[2]);
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