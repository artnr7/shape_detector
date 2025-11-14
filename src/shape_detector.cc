// #include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
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

  cv::Mat img1;
  ImgRead((const char **)argv, img1, cv::ImreadModes::IMREAD_COLOR);

  cv::Mat img2;
  cv::cvtColor(img1, img2, cv::COLOR_BGR2GRAY);
  for (int i = 0; i < img2.rows; i++) {
    for (int j = 0; j < img2.cols; j++) {
      cv::Vec3b pixel =
          img2.at<cv::Vec3b>(i, j);  // Vec3b для 3-х канального uchar
      std::cout << "Pixel (" << i << "," << j << "): B=" << (int)pixel[0]
                << " G=" << (int)pixel[1] << " R=" << (int)pixel[2]
                << std::endl;
    }
    break;
  }

  cv::Mat img;
  cv::threshold(img2, img, 150, 255, cv::THRESH_BINARY);

  auto detector = cv::SimpleBlobDetector::create();

  cv::SimpleBlobDetector::Params params{};
  SetParams(params);
  detector->setParams(params);

  std::vector<cv::KeyPoint> keypoints;
  detector->detect(img, keypoints);

  cv::Mat blob_img;
  cv::drawKeypoints(img, keypoints, blob_img, cv::Scalar(0, 0, 255),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  cv::Mat blur_img;
  cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0);

  cv::Mat sobel_img;
  cv::Sobel(blur_img, sobel_img, CV_64F, 1, 1, 5);

  cv::Mat edges;
  cv::Canny(blur_img, edges, 100, 200, 3, false);
  imshow("Shape Detector", img);
  // imshow("Shape Detector → Blob", blob_img);
  // imshow("Shape Detector → Blur", blur_img);
  // imshow("Shape Detector → Sobel", sobel_img);
  imshow("Shape Detector → Edges", edges);

  cv::waitKey(0);

  cv::destroyAllWindows();

  imwrite("ch_img_1.jpg", img);

  return 0;
}

void SetParams(cv::SimpleBlobDetector::Params &params) {
  params.minThreshold = 10;
  params.maxThreshold = 200;

  params.filterByArea = true;
  params.minArea = 1500;

  params.filterByCircularity = true;
  params.minCircularity = 0.1;

  params.filterByConvexity = true;
  params.minConvexity = 0.87;

  params.filterByInertia = true;
  params.minInertiaRatio = 0.01;

  params.filterByColor = true;
  params.blobColor = 255;
}

void ImgRead(const char **argv, cv::Mat &img,
             const enum cv::ImreadModes &img_read_mode) {
  img = imread(argv[1], img_read_mode);
}