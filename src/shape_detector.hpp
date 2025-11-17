#include <opencv4/opencv2/opencv.hpp>

namespace sd {
#define DEF_WIN_MULT 30

#define BORDER_X 10
#define BORDER_Y 35

#define MAIN_WIN_W (16 * DEF_WIN_MULT * 2)
#define MAIN_WIN_H (9 * DEF_WIN_MULT * 2)
#define H_WIN_W (16 * DEF_WIN_MULT)
#define H_WIN_H (9 * DEF_WIN_MULT)
#define S_WIN_W (16 * DEF_WIN_MULT)
#define S_WIN_H (9 * DEF_WIN_MULT)
#define V_WIN_W (16 * DEF_WIN_MULT)
#define V_WIN_H (9 * DEF_WIN_MULT)

#define MAIN_WIN_X 50
#define MAIN_WIN_Y 50

#define UTIL_WIN_X MAIN_WIN_X
#define UTIL_WIN_Y (MAIN_WIN_Y + MAIN_WIN_H + BORDER_Y)

#define H_WIN_X UTIL_WIN_X
#define H_WIN_Y UTIL_WIN_Y

#define S_WIN_X (UTIL_WIN_X + H_WIN_W + BORDER_X)
#define S_WIN_Y UTIL_WIN_Y

#define V_WIN_X (S_WIN_X + S_WIN_W + BORDER_X)
#define V_WIN_Y UTIL_WIN_Y

void MoveWindows(const std::string &main_w, const std::string &h_w,
                 const std::string &s_w, const std::string &v_w);

void SetParams(cv::SimpleBlobDetector::Params &params);

void ImgReadFirstFile(const char **argv, cv::Mat &img,
                      const enum cv::ImreadModes &img_read_mode);

#define PAUSE 50  // milliseconds
bool MainCycleState();

void ResizeWindows(const std::string &main_win, const std::string &h_w,
                   const std::string &s_w, const std::string &v_w);

void CreateTrackbars(const std::vector<cv::Mat> &channels,
                     const std::string &h_win, const std::string &s_win,
                     const std::string &v_win, int &h_min, int &h_max,
                     int &s_min, int &s_max, int &v_min, int &v_max);

void ChangeTrackbarsValues(int &h_min, int &h_max, int &s_min, int &s_max,
                           int &v_min, int &v_max);

void ShowImages(const std::string &main_win, const std::string &h_win,
                const std::string &s_win, const std::string &v_win,
                cv::Mat &hsv_img, const std::vector<cv::Mat> &channels);

void ChannelsInRange(const std::vector<cv::Mat> &channels,
                     std::vector<cv::Mat> &channels_ranged, int h_min,
                     int h_max, int s_min, int s_max, int v_min, int v_max);

void ChannelsSum(cv::Mat &hsv_bin_sum,
                 const std::vector<cv::Mat> channels_ranged);
}  // namespace sd