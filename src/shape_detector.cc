#include "shape_detector.hpp"

int main(int argc, char** argv) {
  int app_mode = 0;

#ifdef MODE_CONFIG
  app_mode = 0;
#endif
#ifdef MODE_OUTPUT
  app_mode = 1;
#endif

  for (int i = 1; i < argc; ++i) {
    sd::proccessing_img(argv[i], app_mode);
  }

  return 0;
}

void sd::proccessing_img(const std::string& filename, int app_mode) {
  // Primitives
  // src
  cv::Mat img;
  img = imread(filename, cv::IMREAD_COLOR);
  // blured
  cv::Mat blur_img;
  cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0);
  // hsv channels img
  cv::Mat hsv_img;
  cv::cvtColor(blur_img, hsv_img, cv::COLOR_BGR2HSV);
  // mask
  cv::Mat mask_img = cv::Mat::zeros(img.size(), CV_8UC1);
  // colorized mask
  cv::Mat clr_mask_img;
  img.copyTo(clr_mask_img);
  // edges
  cv::Mat edges_img;
  cv::Canny(blur_img, edges_img, 100, 200, 3, false);

  // Main goal value
  cv::Mat sign_detect_res_img = cv::Mat::zeros(img.size(), img.type());

  // Windows
  std::string main_win{"Rawd Sign Detector"};
  std::string h_win{"h channel"};
  std::string s_win{"s channel"};
  std::string v_win{"v channel"};
  std::string hsv_chnls_sum_win{"Channels sum"};
  std::string edges_win{"Canny edges"};
  std::string mask_win{"Mask"};

  if (app_mode == 0) {
    sd::MakeWindows(main_win, h_win, s_win, v_win, hsv_chnls_sum_win, edges_win,
                    mask_win);

    sd::ResizeWindows(main_win, h_win, s_win, v_win, hsv_chnls_sum_win,
                      edges_win, mask_win);
  }
  // Windows----------------------------------------------------------------------

  // HSV Channels
  std::vector<cv::Mat> channels;
  cv::split(hsv_img, channels);

  // HSV channels trackbars
  int h_min = 0, h_max = 0;
  int s_min = 0, s_max = 0;
  int v_min = 0, v_max = 0;

  sd::ChangeTrackbarsValues(h_min, h_max, s_min, s_max, v_min, v_max);

  if (app_mode == 0) {
    sd::CreateTrackbars(channels, h_win, s_win, v_win, h_min, h_max, s_min,
                        s_max, v_min, v_max);
  }

  // Create InRange Channels
  std::vector<cv::Mat> channels_ranged;
  sd::ChannelsRangedCreate(channels, channels_ranged);

  // Main Cycle
  int output_limit = 0;
  static int output_imgs_cnt = 1;

  int clr_mode = 0;
  while (true) {
    if (app_mode == 1) {
      sd::SetColorMode(clr_mode, h_min, h_max);
    }
    sd::ChannelsInRange(channels, channels_ranged, h_min, h_max, s_min, s_max,
                        v_min, v_max);

    cv::Mat hsv_chnls_sum_img;
    sd::ChannelsSum(hsv_chnls_sum_img, channels_ranged);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(hsv_chnls_sum_img, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    sd::FillMask(contours, hierarchy, img, mask_img);

    sd::SetColorizedMask(img, mask_img, clr_mask_img);

    // Cut obj from src, then we have rectangles on the black background
    sd::WriteContoursRect(contours, clr_mask_img, output_imgs_cnt, output_limit,
                          sign_detect_res_img);

    if (app_mode == 0) {
      sd::ShowImages(main_win, sign_detect_res_img, h_win, s_win, v_win,
                     channels_ranged, hsv_chnls_sum_win, hsv_chnls_sum_img,
                     edges_win, edges_img, mask_win, mask_img);

      // Unfortunatelly I shall move windows after their show proccess(every
      // time)
      sd::MoveWindows(main_win, h_win, s_win, v_win, hsv_chnls_sum_win,
                      edges_win, mask_win);
    }

    // App exit
    if (app_mode == 0 && cv::waitKey(PAUSE) == 27) {
      break;
    }
    if (app_mode == 1 && output_limit == COLOR_QTY) {
      break;
    }
    ++output_limit;
  }
  if (app_mode == 0) {
    cv::destroyAllWindows();
  }
}
