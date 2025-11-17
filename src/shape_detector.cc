#include "shape_detector.hpp"

#include <cstddef>
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  (void)argc;

  std::string main_win{"Rawd Sign Detector"};
  std::string img_win{"Raw img win"};
  cv::namedWindow(main_win, cv::WINDOW_NORMAL);
  cv::namedWindow(img_win, cv::WINDOW_NORMAL);

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
  cv::resizeWindow(img_win, MAIN_WIN_W, MAIN_WIN_H);

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

  while (true) {
    sd::ChannelsInRange(channels, channels_ranged, h_min, h_max, s_min, s_max,
                        v_min, v_max);

    cv::Mat hsv_bin_sum;
    sd::ChannelsSum(hsv_bin_sum, channels_ranged);

    sd::ShowImages(main_win, h_win, s_win, v_win, hsv_bin_sum, channels_ranged);
    // ------------

    cv::imshow(edges_win, edges_img);
    cv::moveWindow(edges_win, MAIN_WIN_X + MAIN_WIN_W + BORDER_X, MAIN_WIN_Y);

    // ------------
    cv::Mat hsv_bin_edges_img_sum;
    cv::bitwise_and(hsv_bin_sum, edges_img, hsv_bin_edges_img_sum);
    cv::imshow(hsv_bin_edges_img_sum_win, hsv_bin_edges_img_sum);
    cv::moveWindow(hsv_bin_edges_img_sum_win,
                   MAIN_WIN_X + 2 * MAIN_WIN_W + BORDER_X, MAIN_WIN_Y);
    // ------------

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(hsv_bin_edges_img_sum, contours, hierarchy,
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++) {
      cv::drawContours(img, contours, static_cast<int>(i),
                       cv::Scalar(120, 255, 255), 2, cv::LINE_8, hierarchy, 0);
    }

    cv::imshow(img_win, img);
    cv::moveWindow(img_win, MAIN_WIN_X + 3 * MAIN_WIN_W + BORDER_X,
                   MAIN_WIN_Y + 100);

    sd::MoveWindows(main_win, h_win, s_win, v_win);

    if (cv::waitKey(PAUSE) == 27) {
      break;
    }
  }

  cv::destroyAllWindows();
}
