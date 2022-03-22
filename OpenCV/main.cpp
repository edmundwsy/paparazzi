#include <stdio.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;
using namespace std;
using namespace std::chrono;

void coloryuv_opencv_to_yuv422(Mat image, char *img, int width, int height) {
  CV_Assert(image.depth() == CV_8U);
  CV_Assert(image.channels() == 3);

  int nRows = image.rows;
  int nCols = image.cols;

  int byte_index = 0;
  for (int r = 0; r < nRows; ++r) {
    for (int c = 0; c < nCols; ++c) {
      Vec3b yuv = image.at<Vec3b>(r, c);
      if ((byte_index % 4) == 0) {
        img[byte_index++] = yuv.val[1];  // U
      } else {
        img[byte_index++] = yuv.val[2];  // V
      }
      img[byte_index++] = yuv.val[0];  // Y
    }
  }
}

void colorbgr_opencv_to_yuv422(Mat image, char *img, int width, int height) {
  // Convert to YUV color space
  cvtColor(image, image, COLOR_BGR2YUV);
  // then call the to color function
  coloryuv_opencv_to_yuv422(image, img, width, height);
}

void grayscale_opencv_to_yuv422(Mat image, char *img, int width, int height) {
  CV_Assert(image.depth() == CV_8U);
  CV_Assert(image.channels() == 1);

  int n_rows = image.rows;
  int n_cols = image.cols;

  // If the image is one block in memory we can iterate over it all at once!
  if (image.isContinuous()) {
    n_cols *= n_rows;
    n_rows = 1;
  }

  // Iterate over the image, setting only the Y value
  // and setting U and V to 127
  int    i, j;
  uchar *p;
  int    index_img = 0;
  for (i = 0; i < n_rows; ++i) {
    p = image.ptr<uchar>(i);
    for (j = 0; j < n_cols; j++) {
      img[index_img++] = 127;
      img[index_img++] = p[j];
    }
  }
}

int main() {
  // Mat img_ori = imread("");
  // Mat img_ori = imread("imgs/361876501.jpg");
  Mat img_ori = imread("imgs/376609733.jpg");
  // Mat img_ori = imread("imgs/368709783.jpg");
  // Mat img_ori = imread("imgs/372043080.jpg");
  // Mat img_ori = imread("imgs/347443312.jpg");
  // Mat img_ori = imread("imgs/356009880.jpg");
  rotate(img_ori, img_ori, cv::ROTATE_90_COUNTERCLOCKWISE);
  int width  = img_ori.cols;
  int height = img_ori.rows;

  std::cout << "width " << width << " height " << height << std::endl;

  Mat image, image_tmp, src;
  cvtColor(img_ori, image, COLOR_RGB2BGR);
  blur(image, image, Size(3, 3));
  bilateralFilter(image, image_tmp, 15, 25, 25);
  Scalar low  = Scalar(0, 80, 0);
  Scalar high = Scalar(255, 255, 255);
  cv::inRange(image_tmp, low, high, image);

  high_resolution_clock::time_point tic    = high_resolution_clock::now();
  Mat                               kernel = getStructuringElement(MORPH_RECT, Size(8, 8));
  morphologyEx(image, src, MORPH_OPEN, kernel);

  // std::cout << src.type() << src.channels() << endl;
  // connected component labeling
  RNG rng(42);
  Mat stats, centroids;
  Mat labels     = Mat::zeros(src.size(), src.type());
  int num_labels = connectedComponentsWithStats(src, labels, stats, centroids, 4);
  cout << "found connected components: " << num_labels << endl;

  // Filtering
  int *valid_labels = new (int[num_labels]);
  for (int i = 0; i < num_labels; i++) {
    int width  = stats.at<int>(i, CC_STAT_WIDTH);
    int height = stats.at<int>(i, CC_STAT_HEIGHT);

    if (width < 30 || height < 30 || float(width / height) > 5) {
      valid_labels[i] = 0;
    } else {
      valid_labels[i] = i;
    }
    std::cout << valid_labels[i] << ' ';
  }
  std::cout << std::endl;

  high_resolution_clock::time_point toc       = high_resolution_clock::now();
  duration<double>                  time_span = duration_cast<duration<double>>(toc - tic);
  std::cout << "Duration: " << time_span.count() << " seconds" << std::endl;

  vector<Vec3b> colors(num_labels);
  // generate background color => black
  colors[0] = Vec3b(0, 0, 0);
  // generate region color => random
  for (int i = 1; i < num_labels; i++) {
    colors[i] = Vec3b(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
  }
  // display components, drawing
  Mat dst1 = Mat::zeros(src.size(), CV_8UC3);
  int w    = src.cols;
  int h    = src.rows;
  std::cout << "width " << w << " height " << h << std::endl;
  for (int row = 0; row < h; row++) {
    for (int col = 0; col < w; col++) {
      int label = labels.at<int>(row, col);
      if (valid_labels[label] == 0) {
        continue;
      }
      dst1.at<Vec3b>(row, col) = colors[label];
    }
  }

  // static and drawing
  for (int i = 1; i < num_labels; i++) {
    Vec2d pt     = centroids.at<Vec2d>(i, 0);
    int   x      = stats.at<int>(i, CC_STAT_LEFT);
    int   y      = stats.at<int>(i, CC_STAT_TOP);
    int   width  = stats.at<int>(i, CC_STAT_WIDTH);
    int   height = stats.at<int>(i, CC_STAT_HEIGHT);
    int   area   = stats.at<int>(i, CC_STAT_AREA);
    printf("area : %d, center point(%.2f, %.2f)\n", area, pt[0], pt[1]);       //面积信息
    circle(dst1, Point(pt[0], pt[1]), 2, Scalar(0, 0, 255), -1, 8, 0);         //中心点坐标
    rectangle(dst1, Rect(x, y, width, height), Scalar(255, 0, 255), 1, 8, 0);  //外接矩形
  }

  imwrite("./rst/test.jpg", image);
  imwrite("./rst/mor.jpg", src);
  imwrite("./rst/origin.jpg", img_ori);
  imwrite("./rst/connected.jpg", dst1);
}