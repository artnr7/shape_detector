#include "shape_detector.hpp"

#include <opencv2/core/hal/interface.h>

#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
  // Main goal value
  cv::Mat sign_detect_res_img;
  // Primitives
  cv::Mat img;
  sd::ImgReadFirstFile((const char **)argv, img, cv::IMREAD_COLOR);
  cv::Mat blur_img;
  cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0);
  cv::Mat edges_img;
  cv::Canny(blur_img, edges_img, 100, 200, 3, false);
  cv::Mat hsv_img;
  cv::cvtColor(blur_img, hsv_img, cv::COLOR_BGR2HSV);

  // Windows
  std::string main_win{"Rawd Sign Detector"};
  cv::namedWindow(main_win, cv::WINDOW_NORMAL);

  std::string h_win{"h channel"};
  std::string s_win{"s channel"};
  std::string v_win{"v channel"};

  cv::namedWindow(h_win, cv::WINDOW_NORMAL);
  cv::namedWindow(s_win, cv::WINDOW_NORMAL);
  cv::namedWindow(v_win, cv::WINDOW_NORMAL);
  // sd::MakeWindows(h_win, s_win, v_win);

  std::string hsv_chnls_sum_win{"Channels sum"};
  cv::namedWindow(hsv_chnls_sum_win, cv::WINDOW_NORMAL);

  std::string edges_win{"Canny edges"};
  cv::namedWindow(edges_win, cv::WINDOW_NORMAL);

  std::string hsv_chnls_and_edges_sum_win{"Edges and channels"};
  cv::namedWindow(hsv_chnls_and_edges_sum_win, cv::WINDOW_NORMAL);

  sd::ResizeWindows(main_win, h_win, s_win, v_win, hsv_chnls_sum_win, edges_win,
                    hsv_chnls_and_edges_sum_win);
  // Windows----------------------------------------------------------------------

  // HSV Channels
  std::vector<cv::Mat> channels;
  cv::split(hsv_img, channels);

  // HSV channels trackbars
  int h_min = 0, h_max = 0;
  int s_min = 0, s_max = 0;
  int v_min = 0, v_max = 0;

  sd::ChangeTrackbarsValues(h_min, h_max, s_min, s_max, v_min, v_max);
  sd::CreateTrackbars(channels, h_win, s_win, v_win, h_min, h_max, s_min, s_max,
                      v_min, v_max);

  // Create InRange Channels
  std::vector<cv::Mat> channels_ranged;
  sd::ChannelsRangedCreate(channels, channels_ranged);

  // Main Cycle
  while (true) {
    sd::ChannelsInRange(channels, channels_ranged, h_min, h_max, s_min, s_max,
                        v_min, v_max);

    cv::Mat hsv_chnls_sum_img;
    sd::ChannelsSum(hsv_chnls_sum_img, channels_ranged);

    cv::Mat hsv_chnls_and_edges_sum_img;
    cv::bitwise_and(hsv_chnls_sum_img, edges_img, hsv_chnls_and_edges_sum_img);

    sd::FindAndDrawContours(hsv_chnls_and_edges_sum_img, img,
                            sign_detect_res_img);

    sd::ShowImages(main_win, sign_detect_res_img, h_win, s_win, v_win,
                   channels_ranged, hsv_chnls_sum_win, hsv_chnls_sum_img,
                   edges_win, edges_img, hsv_chnls_and_edges_sum_win,
                   hsv_chnls_and_edges_sum_img);

    // Unfortunatelly I shall move windows after their show proccess (evry
    // time)

    sd::MoveWindows(main_win, h_win, s_win, v_win, hsv_chnls_sum_win, edges_win,
                    hsv_chnls_and_edges_sum_win);

    // App exit
    if (cv::waitKey(PAUSE) == 27) {
      break;
    }
  }

  cv::destroyAllWindows();
  (void)argc;
  return 0;
}
