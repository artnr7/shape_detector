#include "shape_detector.hpp"

void sd::MakeWindows(const std::string& main_win, const std::string& h_win,
                     const std::string& s_win, const std::string& v_win,
                     const std::string& hsv_chnls_sum_win,
                     const std::string& edges_win,
                     const std::string& mask_win) {
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

#ifdef MASK
  cv::namedWindow(mask_win, cv::WINDOW_NORMAL);
#endif

#ifndef HSV
  (void)hsv_chnls_sum_win;
#endif

#ifndef EDGES
  (void)edges_win;
#endif

#ifndef MASK
  (void)mask_win;
#endif
}

void sd::ResizeWindows(const std::string& main_win, const std::string& h_win,
                       const std::string& s_win, const std::string& v_win,
                       const std::string& hsv_chnls_sum_win,
                       const std::string& edges_win,
                       const std::string& mask_win) {
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

#ifdef MASK
  cv::resizeWindow(mask_win, MAIN_WIN_W, MAIN_WIN_H);
#endif

#ifndef HSV
  (void)hsv_chnls_sum_win;
#endif

#ifndef EDGES
  (void)edges_win;
#endif

#ifndef MASK
  (void)mask_win;
#endif
}

// Какую матрицу изображения где показывать
void sd::ShowImages(const std::string& main_win,
                    const cv::Mat& sign_detect_res_img,
                    const std::string& h_win, const std::string& s_win,
                    const std::string& v_win,
                    const std::vector<cv::Mat> channels_ranged,
                    const std::string& hsv_chnls_sum_win,
                    const cv::Mat& hsv_chnls_sum_img,
                    const std::string& edges_win, const cv::Mat& edges_img,
                    const std::string& mask_win, const cv::Mat& mask_img) {
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

#ifdef MASK
  cv::imshow(mask_win, mask_img);
#endif

#ifndef HSV
  (void)hsv_chnls_sum_win;
  (void)hsv_chnls_sum_img;
#endif

#ifndef EDGES
  (void)edges_win;
  (void)edges_img;
#endif

#ifndef MASK
  (void)mask_win;
  (void)mask_img;
#endif
}

void sd::MoveWindows(const std::string& main_win, const std::string& h_win,
                     const std::string& s_win, const std::string& v_win,
                     const std::string& hsv_chnls_sum_win,
                     const std::string& edges_win,
                     const std::string& mask_win) {
  int win_cnt = 1;
  cv::moveWindow(main_win, MAIN_WIN_X, MAIN_WIN_Y);

  cv::moveWindow(h_win, H_WIN_X, H_WIN_Y);
  cv::moveWindow(s_win, S_WIN_X, S_WIN_Y);
  cv::moveWindow(v_win, V_WIN_X, V_WIN_Y);

#ifdef HSV
  cv::moveWindow(hsv_chnls_sum_win,
                 MAIN_WIN_X + win_cnt++ * MAIN_WIN_W + BORDER_X, MAIN_WIN_Y);
#endif

#ifdef EDGES
  cv::moveWindow(edges_win, MAIN_WIN_X + win_cnt++ * MAIN_WIN_W + BORDER_X,
                 MAIN_WIN_Y);
#endif

#ifdef MASK
  cv::moveWindow(mask_win, MAIN_WIN_X + win_cnt++ * MAIN_WIN_W + BORDER_X,
                 MAIN_WIN_Y);
#endif

#ifndef HSV
  (void)hsv_chnls_sum_win;
#endif

#ifndef EDGES
  (void)edges_win;
#endif

#ifndef MASK
  (void)mask_win;
#endif

  ++win_cnt;
}

void sd::ChangeTrackbarsValues(int& h_min, int& h_max, int& s_min, int& s_max,
                               int& v_min, int& v_max) {
  h_min = RED_MIN;
  h_max = RED_MAX;
  s_min = 65;
  s_max = 255;
  v_min = 65;
  v_max = 255;
}

void sd::CreateTrackbars(const std::vector<cv::Mat>& channels,
                         const std::string& h_win, const std::string& s_win,
                         const std::string& v_win, int& h_min, int& h_max,
                         int& s_min, int& s_max, int& v_min, int& v_max) {
  int h_maxv = 179;
  int sv_max = 255;
  double min = 0, max = 0;

  cv::minMaxLoc(channels[0], &min, &max);
  cv::createTrackbar("Hmin", h_win, &h_min, h_maxv);
  cv::createTrackbar("Hmax", h_win, &h_max, h_maxv);

  cv::minMaxLoc(channels[1], &min, &max);
  cv::createTrackbar("Smin", s_win, &s_min, sv_max);
  cv::createTrackbar("Smax", s_win, &s_max, sv_max);

  cv::minMaxLoc(channels[2], &min, &max);
  cv::createTrackbar("Vmin", v_win, &v_min, sv_max);
  cv::createTrackbar("Vmax", v_win, &v_max, sv_max);
}

void sd::ChannelsRangedCreate(const std::vector<cv::Mat>& channels,
                              std::vector<cv::Mat>& channels_ranged) {
  for (size_t i = 0; i < channels.size(); ++i) {
    channels_ranged.push_back({});
  }
}

void sd::SetColorMode(int& clr_mode, int& h_min, int& h_max) {
  switch (clr_mode) {
    case sd::RED:
      h_min = RED_MIN;
      h_max = RED_MAX;
      break;
    case sd::BLUE:
      h_min = BLUE_MIN;
      h_max = BLUE_MAX;
      break;
    case sd::YELLOW:
      h_min = YELLOW_MIN;
      h_max = YELLOW_MAX;
      break;
  }
  clr_mode %= COLOR_QTY;
  ++clr_mode;
}

void sd::ChannelsInRange(const std::vector<cv::Mat>& channels,
                         std::vector<cv::Mat>& channels_ranged, int h_min,
                         int h_max, int s_min, int s_max, int v_min,
                         int v_max) {
  cv::inRange(channels[0], h_min, h_max, channels_ranged[0]);
  cv::inRange(channels[1], s_min, s_max, channels_ranged[1]);
  cv::inRange(channels[2], v_min, v_max, channels_ranged[2]);
}

// Ищет контуры в search_src, после рисует их на img_copy и заливает их изнутри
void sd::FillMask(std::vector<std::vector<cv::Point>>& contours,
                  std::vector<cv::Vec4i>& hierarchy, const cv::Mat& src,
                  cv::Mat& mask) {
  mask = cv::Mat::zeros(src.size(), CV_8UC1);
  for (size_t i = 0; i < contours.size(); i++) {
    cv::drawContours(mask, contours, static_cast<int>(i),
                     cv::Scalar(255, 255, 255), -1, cv::LINE_8, hierarchy, 0);
  }
}

void sd::ChannelsSum(cv::Mat& hsv_bin_sum,
                     const std::vector<cv::Mat> channels_ranged) {
  cv::bitwise_and(channels_ranged[0], channels_ranged[1], hsv_bin_sum);
  cv::bitwise_and(channels_ranged[2], hsv_bin_sum, hsv_bin_sum);
}

void sd::SetColorizedMask(const cv::Mat& img, cv::Mat mask_img,
                          cv::Mat& clr_mask_img) {
  cv::cvtColor(mask_img, mask_img, cv::COLOR_GRAY2BGR);
  img.copyTo(clr_mask_img);
  cv::bitwise_and(clr_mask_img, mask_img, clr_mask_img);
}

void sd::WriteContoursRect(const std::vector<std::vector<cv::Point>> contours,
                           const cv::Mat& src, int& output_imgs_cnt,
                           int& output_limit, cv::Mat& dst) {
  dst.setTo(cv::Scalar(0, 0, 0));

  for (const auto& el : contours) {
    double area = cv::contourArea(el);
    double perimeter = cv::arcLength(el, true);

    if (el.size() >= 5) {
      double circularity = 4 * CV_PI * (area / (perimeter * perimeter));
      if (circularity < 0.6) {
        continue;
      }
      cv::RotatedRect ellipse = cv::fitEllipse(el);
      double ratio = ellipse.size.width / ellipse.size.height;
      if (ratio < 0.6 || ratio > 1.4) {
        continue;
      }
    } else {
      std::vector<cv::Point> approx;
      cv::approxPolyDP(el, approx, 0.02 * perimeter, true);

      if (!((approx.size() == 3 || approx.size() == 4) && area > 15)) {
        continue;
      }
    }

    cv::Rect bbox = cv::boundingRect(el);

    // Копируем область из исходного изображения
    cv::Mat object_roi = src(bbox).clone();

    if (!(object_roi.rows > MIN_IMG_AX_SIZE &&
          object_roi.cols > MIN_IMG_AX_SIZE)) {
      continue;
    }
    // Вставляем объект в финальное изображение
    object_roi.copyTo(dst(bbox));

    if (!(output_limit < COLOR_QTY)) {
      continue;
    }
    cv::imwrite("output/" + std::to_string(output_imgs_cnt++) + ".jpg",
                object_roi);
  }
}
