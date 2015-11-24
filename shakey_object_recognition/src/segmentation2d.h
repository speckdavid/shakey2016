#ifndef SHAKEY_SEGMENTATION2D_H_
#define SHAKEY_SEGMENTATION2D_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

class Segmentation2d
{
public:
  Segmentation2d() {}

  Mat houghlinesP(Mat& src)
  {
    Mat dst, cdst;
    Canny(src, dst, 50, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);
    vector<Vec4i> lines;
    HoughLinesP(dst, lines, 1, CV_PI/180, 70, 50, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
    return cdst;
  }

  Mat adaptiveThresholding(Mat& src)
  {
    Mat dst;
    cvtColor(src, dst, CV_BGR2GRAY);
    //adaptiveThreshold(dst, dst, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 75, 10);
    adaptiveThreshold(dst, dst, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, 2);
    cvtColor(dst, dst, CV_GRAY2BGR);
    return dst;
  }

  Mat contours(Mat& src)
  { 
    Mat dst;
    cvtColor(src, dst, CV_BGR2GRAY);
    Canny(dst, dst, 100, 200, 3);
    /// Find contours  
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    // Possible to get only conner points
    findContours( dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    /// Draw contours
    Mat drawing = Mat::zeros( dst.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }
    return drawing;
  }

  Mat dualPic(Mat& img1, Mat& img2)
  {
    Size sz1 = img1.size();
    Size sz2 = img2.size();
    Mat finalImg(sz1.height, sz1.width+sz2.width, CV_8UC3);
    Mat left(finalImg, Rect(0, 0, sz1.width, sz1.height));
    img1.copyTo(left);
    Mat right(finalImg, Rect(sz1.width, 0, sz2.width, sz2.height));
    img2.copyTo(right);  
    return finalImg;
  }

};

#endif  // SHAKEY_2DSEGMENTATION_H_
