#include <opencv4/opencv2/opencv.hpp>

namespace sd {
#define DEF_WIN_MULT 30

#define BORDER_X 10
#define BORDER_Y 35

#define MAIN_WIN_W (16 * DEF_WIN_MULT * 2)
#define MAIN_WIN_H (9 * DEF_WIN_MULT * 2)
#define UTIL_WIN_W (MAIN_WIN_W / 2)
#define UTIL_WIN_H (MAIN_WIN_H / 2)
#define H_WIN_W UTIL_WIN_W
#define H_WIN_H UTIL_WIN_H
#define S_WIN_W UTIL_WIN_W
#define S_WIN_H UTIL_WIN_H
#define V_WIN_W UTIL_WIN_W
#define V_WIN_H UTIL_WIN_H

#define MAIN_WIN_X 50
#define MAIN_WIN_Y 100

#define UTIL_WIN_X MAIN_WIN_X
#define UTIL_WIN_Y (MAIN_WIN_Y + MAIN_WIN_H + BORDER_Y)

#define H_WIN_X UTIL_WIN_X
#define H_WIN_Y UTIL_WIN_Y

#define S_WIN_X (UTIL_WIN_X + H_WIN_W + BORDER_X)
#define S_WIN_Y UTIL_WIN_Y

#define V_WIN_X (S_WIN_X + S_WIN_W + BORDER_X)
#define V_WIN_Y UTIL_WIN_Y

// color primitives
enum clr { RED = 1, BLUE, YELLOW };

#define RED_MIN 165
#define RED_MAX 179

#define BLUE_MIN 95
#define BLUE_MAX 145

#define YELLOW_MIN 165
#define YELLOW_MAX 179

void ResizeWindows(const std::string &main_win, const std::string &h_win,
                   const std::string &s_win, const std::string &v_win,
                   const std::string &hsv_chnls_sum_win,
                   const std::string &edges_win,
                   const std::string &hsv_chnls_and_edges_sum_win,
                   const std::string &mask_win);

void ShowImages(const std::string &main_win, const cv::Mat &sign_detect_res_img,
                const std::string &h_win, const std::string &s_win,
                const std::string &v_win,
                const std::vector<cv::Mat> channels_ranged,
                const std::string &hsv_chnls_sum_win,
                const cv::Mat &hsv_chnls_sum_img, const std::string &edges_win,
                const cv::Mat &edges_img,
                const std::string &hsv_chnls_and_edges_sum_win,
                const cv::Mat &hsv_chnls_and_edges_sum_img,
                const std::string &mask_win, const cv::Mat &mask_img);

void MoveWindows(const std::string &main_win, const std::string &h_win,
                 const std::string &s_win, const std::string &v_win,
                 const std::string &hsv_chnls_sum_win,
                 const std::string &edges_win,
                 const std::string &hsv_chnls_and_edges_sum_win,
                 const std::string &mask_win);

void SetParams(cv::SimpleBlobDetector::Params &params);

void ImgReadFirstFile(const char **argv, cv::Mat &img,
                      const enum cv::ImreadModes &img_read_mode);

#define PAUSE 150  // milliseconds
bool MainCycleState();

void CreateTrackbars(const std::vector<cv::Mat> &channels,
                     const std::string &h_win, const std::string &s_win,
                     const std::string &v_win, int &h_min, int &h_max,
                     int &s_min, int &s_max, int &v_min, int &v_max);

void ChangeTrackbarsValues(int &h_min, int &h_max, int &s_min, int &s_max,
                           int &v_min, int &v_max);

void ChannelsInRange(const std::vector<cv::Mat> &channels,
                     std::vector<cv::Mat> &channels_ranged, int h_min,
                     int h_max, int s_min, int s_max, int v_min, int v_max);

void ChannelsSum(cv::Mat &hsv_bin_sum,
                 const std::vector<cv::Mat> channels_ranged);

void MakeWindows(const std::string &main_win, const std::string &h_win,
                 const std::string &s_win, const std::string &v_win,
                 const std::string &hsv_chnls_sum_win,
                 const std::string &edges_win,
                 const std::string &hsv_chnls_and_edges_sum_win,
                 const std::string &mask_win);

void FindAndFillContours(const cv::Mat &find_src, const cv::Mat &img,
                         cv::Mat &img_copy);
void ChannelsRangedCreate(const std::vector<cv::Mat> &channels,
                          std::vector<cv::Mat> &channels_ranged);
void SetColorizedMask(const cv::Mat &img, cv::Mat mask_img,
                      cv::Mat &clr_mask_img);

void WriteContoursRect(const std::vector<std::vector<cv::Point>> contours,
                           const cv::Mat &src, int &i, int &j, cv::Mat &dst);
}  // namespace sd