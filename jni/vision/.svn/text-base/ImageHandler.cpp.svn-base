#include "ImageHandler.h"

  void sample(cv::Mat &im, double x, double y, double &result) {
    const int lx = (int)x;
    const int ly = (int)y;
    x -= lx;
    y -= ly;

    result = (double) ((1-y)*((1-x)*im.at<float>(ly, lx) + x*im.at<float>(ly, lx+1)) + y * ((1-x)*im.at<float>(ly+1, lx) + x*im.at<float>(ly+1, lx+1)));
  }

  void sample(cv::Mat &im, double x, double y, unsigned char &result) {
    const int lx = (int)x;
    const int ly = (int)y;
    x -= lx;
    y -= ly;

    result = (unsigned char) ((1-y)*((1-x)*im.at<unsigned char>(ly, lx) + x*im.at<unsigned char>(ly, lx+1)) + y * ((1-x)*im.at<unsigned char>(ly+1, lx) + x*im.at<unsigned char>(ly+1, lx+1)));
  }

  int transform_image(cv::Mat &in, cv::Mat &out, const Eigen::Matrix2d& M, const Eigen::Vector2d& inOrig, const Eigen::Vector2d& outOrig, double defaultValue) {
    const int w = out.cols, h = out.rows, iw = in.cols, ih = in.rows; 
    Eigen::Vector2d across(M(0,0), M(1,0));
    Eigen::Vector2d down(M(0,1), M(1,1));

    Eigen::Vector2d p0 = inOrig - M*outOrig;
        
    // ul --> p0
    // ur --> w*across + p0
    // ll --> h*down + p0
    // lr --> w*across + h*down + p0
    double min_x = p0(0), min_y = p0(1);
    double max_x = min_x, max_y = min_y;
   
    // Minimal comparisons needed to determine bounds
    if (across(0) < 0)
      min_x += w*across(0);
    else
      max_x += w*across(0);
    if (down(0) < 0)
      min_x += h*down(0);
    else
      max_x += h*down(0);
    if (across(1) < 0)
      min_y += w*across(1);
    else
      max_y += w*across(1);
    if (down(1) < 0)
      min_y += h*down(1);
    else
      max_y += h*down(1);
   
    // This gets from the end of one row to the beginning of the next
    Eigen::Vector2d carriage_return = down - w*across;

    //If the patch being extracted is completely in the image then no 
    //check is needed with each point.
    if (min_x >= 0 && min_y >= 0 && max_x < iw-1 && max_y < ih-1) {
      Eigen::Vector2d p = p0;
      for (int i=0; i<h; ++i, p+=carriage_return)
        for (int j=0; j<w; ++j, p+=across) {
          if(out.type() == CV_8UC1) {
            unsigned char cvalue;
            sample(in,p(0),p(1),cvalue);
            out.at<unsigned char>(i, j) = cvalue;
          } else if(out.type() == CV_32FC1) {
            double cvalue;
            sample(in,p(0),p(1),cvalue);
            out.at<float>(i, j) = (float) cvalue;
          } else {
            std::cerr << "Not valid type in ImageHandler.h" << std::endl;
            exit(-1);
          }
        }
      return 0;
    } else { // Check each source location
      // Store as doubles to avoid conversion cost for comparison
      const float x_bound = iw-1;
      const float y_bound = ih-1;
      int count = 0;
      Eigen::Vector2d p = p0;
      for (int i=0; i<h; ++i, p+=carriage_return) {
        for (int j=0; j<w; ++j, p+=across) {
          //Make sure that we are extracting pixels in the image
          if (0 <= p(0) && 0 <= p(1) &&  p(0) < x_bound && p(1) < y_bound) {
            if(out.type() == CV_8UC1) {
              unsigned char cvalue;
              sample(in,p(0),p(1),cvalue);
              out.at<unsigned char>(i, j) = cvalue;
            } else if(out.type() == CV_32FC1) {
              double cvalue;
              sample(in,p(0),p(1),cvalue);
              out.at<float>(i, j) = (float) cvalue;
            } else {
              std::cerr << "Not valid type in ImageHandler.h" << std::endl;
              exit(-1);
            }
          } else {
            if(out.type() == CV_8UC1)
              out.at<unsigned char>(i, j) = (unsigned char) defaultValue;
            else if(out.type() == CV_32FC1)
              out.at<float>(i, j) = (float) defaultValue;
            else {
              std::cerr << "Not valid type in ImageHandler.h" << std::endl;
              exit(-1);
            }
            ++count;
          }
        }
      }
      return count;
    }
  }

  void copy(cv::Mat &in, cv::Mat &out, int w, int h, int col_i, int row_i) {
    cv::Mat image_roi = in(cv::Rect(row_i, col_i,h,w));
    image_roi.copyTo(out);
  }

	bool in_image_with_border(cv::Mat& img, int px, int py, int border) {
		return px >=border && py >=border && px < img.cols - border && py < img.rows - border;
	}

  double FindShiTomasiScoreAtPoint(cv::Mat img, int nsize, int px, int py) {
    double dXX = 0;
    double dYY = 0;
    double dXY = 0;

    int startx, starty;
    int endx, endy;
    int cx, cy;

    /*Get box pixels*/
    startx = px - nsize;
    starty = py - nsize;
    endx = px + nsize;
    endy = py + nsize;

    for(cy = starty; cy<=endy; cy++)
     for(cx = startx; cx<=endx; cx++){
       double dx = (double)(img.at<unsigned char>(cy, cx + 1) - img.at<unsigned char>(cy, cx - 1));
       double dy = (double)(img.at<unsigned char>(cy + 1, cx) - img.at<unsigned char>(cy - 1, cx));
       dXX += dx*dx;
       dYY += dy*dy;
       dXY += dx*dy;
    }

    int nPixels = (endx - startx + 1)*(endy - starty + 1);
    dXX = dXX / (2.0 * nPixels);
    dYY = dYY / (2.0 * nPixels);
    dXY = dXY / (2.0 * nPixels);

    // Find and return smaller eigenvalue:
    return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
  }
