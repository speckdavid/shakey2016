#ifndef SHAKEY_HOUGHLINESP_H_
#define SHAKEY_HOUGHLINESP_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class HoughlinesP
{
public:
  HoughlinesP() {}

  Mat houghlinesP(Mat& src)
  {
    Mat dst, cdst;
    Canny(src, dst, 50, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);
    vector<Vec4i> lines;
    HoughLinesP(dst, lines, 1, CV_PI/180, 70, 70, 75 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
    return cdst;
  }

  Mat houghlinesPDualPic(Mat& src)
  {
    Mat himg = houghlinesP(src);
    Size sz1 = src.size();
    Size sz2 = himg.size();
    Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
    Mat left(im3, Rect(0, 0, sz1.width, sz1.height));
    src.copyTo(left);
    Mat right(im3, Rect(sz1.width, 0, sz2.width, sz2.height));
    himg.copyTo(right);  
    return im3;
  }

};

#endif  // SHAKEY_HOUGHLINESP_H_
