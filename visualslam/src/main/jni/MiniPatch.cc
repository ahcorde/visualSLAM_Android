#include "MiniPatch.h"

using namespace std;

// Scoring function
inline int MiniPatch::SSDAtPoint(cv::Mat &im, int icol, int irow) {
  if(!in_image_with_border(im, icol, irow, mnHalfPatchSize))
    return mnMaxSSD + 1;
  int basecol = icol - mnHalfPatchSize;
  int baserow = irow - mnHalfPatchSize;
  int nRows = mimOrigPatch.rows;
  int nCols = mimOrigPatch.cols;
  unsigned char *imagepointer;
  unsigned char *templatepointer;
  int nDiff;
  int nSumSqDiff = 0;

  for(int nRow = 0; nRow < nRows; nRow++) {
    imagepointer = im.ptr<unsigned char>(baserow + nRow);
    templatepointer = mimOrigPatch.ptr<unsigned char>(nRow);
    for(int nCol = 0; nCol < nCols; nCol++)	{
      nDiff = imagepointer[basecol + nCol] - templatepointer[nCol];
      nSumSqDiff += nDiff * nDiff;
    };
  };
  return nSumSqDiff;
}

// Find a patch by searching at FAST corners in an input image
// If available, a row-corner LUT is used to speed up search through the
// FAST corners
bool MiniPatch::FindPatch(Eigen::Vector2d &irPos, cv::Mat &im, int nRange, vector<Eigen::Vector2d> &vCorners,  std::vector<int> *pvRowLUT)
{
  Eigen::Vector2d irBest(0,0);
  int nBestSSD = mnMaxSSD + 1;
  Eigen::Vector2d irBBoxTL(irPos(0) - nRange, irPos(1) - nRange);
  Eigen::Vector2d irBBoxBR(irPos(0) + nRange, irPos(1) + nRange);

  vector<Eigen::Vector2d>::iterator i;
  if(!pvRowLUT) {
    for(i = vCorners.begin(); i!=vCorners.end(); i++)
      if((*i)(1) >= irBBoxTL(1)) break;
  } else {
    int nTopRow = irBBoxTL(1);
    if(nTopRow < 0)
      nTopRow = 0;
    if(nTopRow >= (int) pvRowLUT->size())
      nTopRow = (int) pvRowLUT->size() - 1;
    i = vCorners.begin() + (*pvRowLUT)[nTopRow];
  }
  
  for(; i!=vCorners.end(); i++) {
    if((*i)(0) < irBBoxTL(0)  || (*i)(0) > irBBoxBR(0))
      continue;
    if((*i)(1) > irBBoxBR(1))
      break;
    int nSSD = SSDAtPoint(im, (int)(*i)(0), (int)(*i)(1));
    
    if(nSSD < nBestSSD)	{
      irBest(0) = (*i)(0);
      irBest(1) = (*i)(1);
      nBestSSD = nSSD;
    }
  }
  if(nBestSSD < mnMaxSSD) {
    irPos = irBest;
    return true;
  } else
    return false;
}

// Define the patch from an input image
void MiniPatch::SampleFromImage(Eigen::Vector2d irPos, cv::Mat &im)
{
  assert(in_image_with_border(im, (int)irPos(0), (int)irPos(1), mnHalfPatchSize));
  mimOrigPatch.create(2 * mnHalfPatchSize + 1, 2 * mnHalfPatchSize + 1, CV_8UC1);
  copy(im,
		  mimOrigPatch,
		  mimOrigPatch.cols,
		  mimOrigPatch.rows,
		  (int)(irPos(1) - mimOrigPatch.cols / 2),
		  (int)(irPos(0) - mimOrigPatch.rows / 2));
}

// Static members
int MiniPatch::mnHalfPatchSize = 4;
int MiniPatch::mnRange = 10;
int MiniPatch::mnMaxSSD = 9999;













