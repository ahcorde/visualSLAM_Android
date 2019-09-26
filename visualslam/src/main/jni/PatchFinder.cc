#include "PatchFinder.h"
#include "KeyFrame.h"

using namespace std;

Eigen::Vector3d myPatchFinderUnproject(Eigen::Vector2d& v) {
    Eigen::Vector3d result;
    result(0) = v(0);
    result(1) = v(1);
    result(2) = 1.0;

    return result;
}

PatchFinder::PatchFinder(int nPatchSize) {
  mnPatchSize = nPatchSize;
  mirCenter(0) = nPatchSize/2;
  mirCenter(1) = nPatchSize/2;
  int nMaxSSDPerPixel = 500; // Pretty arbitrary... could make a GVar out of this.
  mnMaxSSD = mnPatchSize * mnPatchSize * nMaxSSDPerPixel;
  // Populate the speed-up caches with bogus values:

  mm2LastWarpMatrix = 9999.9 * Eigen::MatrixXd::Identity(2, 2);

  mpLastTemplateMapPoint = NULL;

  this->mimTemplate.create(mnPatchSize, mnPatchSize, CV_8UC1);
}

// Find the warping matrix and search level
int PatchFinder::CalcSearchLevelAndWarpMatrix(MapPoint &p, mySE3 se3CFromW, Eigen::Matrix2d &m2CamDerivs) {
  // Calc point pos in new view camera frame
  // Slightly dumb that we re-calculate this here when the tracker's already done this!
  Eigen::Vector3d v3Cam = se3CFromW * p.v3WorldPos;

  double dOneOverCameraZ = 1.0 / v3Cam(2);
  // Project the source keyframe's one-pixel-right and one-pixel-down vectors into the current view
  Eigen::Vector3d v3MotionRight = se3CFromW.get_rotation() * p.v3PixelRight_W;
  Eigen::Vector3d v3MotionDown = se3CFromW.get_rotation() * p.v3PixelDown_W;

  // Calculate in-image derivatives of source image pixel motions:
  Eigen::Vector2d aux1 = m2CamDerivs * (v3MotionRight.segment(0,2) - v3Cam.segment(0,2) * v3MotionRight(2) * dOneOverCameraZ) * dOneOverCameraZ;;
  Eigen::Vector2d aux2 = m2CamDerivs * (v3MotionDown.segment(0,2) - v3Cam.segment(0,2) * v3MotionDown(2) * dOneOverCameraZ) * dOneOverCameraZ;;

  mm2WarpInverse( 0, 0 )= aux1[0];
  mm2WarpInverse( 0, 1 )= aux2[0];
  mm2WarpInverse( 1, 0 )= aux1[1];
  mm2WarpInverse( 1, 1 )= aux2[1];

  double dDet = mm2WarpInverse.determinant();//mm2WarpInverse[0][0] * mm2WarpInverse[1][1] - mm2WarpInverse[0][1] * mm2WarpInverse[1][0];
  mnSearchLevel = 0;

  // This warp matrix is likely not appropriate for finding at level zero, which is
  // the level at which it has been calculated. Vary the search level until the
  // at that level would be appropriate (does not actually modify the matrix.)
  while(dDet > 3 && mnSearchLevel < LEVELS-1){
    mnSearchLevel++;
    dDet *= 0.25;
  }

  // Some warps are inappropriate, e.g. too near the camera, too far, or reflected,
  // or zero area.. reject these!
  if(dDet > 3 || dDet < 0.25) {
    mbTemplateBad = true;
    return -1;
  } else
    return mnSearchLevel;
}

// This is just a convenience function wich caluclates the warp matrix and generates
// the template all in one call.
void PatchFinder::MakeTemplateCoarse(MapPoint &p, mySE3 se3CFromW, Eigen::Matrix2d& m2CamDerivs)
{
  CalcSearchLevelAndWarpMatrix(p, se3CFromW, m2CamDerivs);
  MakeTemplateCoarseCont(p);
}

// This function generates the warped search template.
void PatchFinder::MakeTemplateCoarseCont(MapPoint &p)
{
  // Get the warping matrix appropriate for use with transform...
  Eigen::Matrix2d m2 = mm2WarpInverse.inverse() * LevelScale(mnSearchLevel);
  // m2 now represents the number of pixels in the source image for one
  // pixel of template image
  
  // Optimisation: Don't re-gen the coarse template if it's going to be substantially the 
  // same as was made last time. This saves time when the camera is not moving. For this, 
  // check that (a) this patchfinder is still working on the same map point and (b) the 
  // warping matrix has not changed much.
  
  bool bNeedToRefreshTemplate = false;
  if(&p != mpLastTemplateMapPoint)
    bNeedToRefreshTemplate = true;
    // Still the same map point? Then compare warping matrix..

  for(int i=0; !bNeedToRefreshTemplate && i<2; i++) {
    Eigen::Vector2d v2Diff = m2.block(0,i,2,1) - mm2LastWarpMatrix.block(0,i,2,1);

    const double dRefreshLimit = 0.07;  // Sort of works out as half a pixel displacement in src img
    if(v2Diff.dot(v2Diff) > dRefreshLimit * dRefreshLimit)
	    bNeedToRefreshTemplate = true;
  }
  
  // Need to regen template? Then go ahead.
  if(bNeedToRefreshTemplate) {
    int nOutside;  // Use transform to warp the patch according the the warping matrix m2
                   // This returns the number of pixels outside the source image hit, which should be zero.
    nOutside = transform_image(p.pPatchSourceKF->aLevels[p.nSourceLevel].im, mimTemplate,  m2, p.irCenter, mirCenter);

    //cout << "nOutside es " << nOutside << endl;
    //cout << "nOutside2 es " << nOutside2 << endl;
  
    if(nOutside)
      mbTemplateBad = true;
    else
      mbTemplateBad = false;
    
    MakeTemplateSums();
    
    // Store the parameters which allow us to determine if we need to re-calculate
    // the patch next time round.
    mpLastTemplateMapPoint = &p;
    mm2LastWarpMatrix = m2;
  }
}

// This makes a template without warping. Used for epipolar search, where we don't really know 
// what the warping matrix should be. (Although to be fair, I should do rotation for epipolar,
// which we could approximate without knowing patch depth!)
void PatchFinder::MakeTemplateCoarseNoWarp(KeyFrame &k, int nLevel, int irLevelPos0, int irLevelPos1) {
  mnSearchLevel = nLevel;
  if(!in_image_with_border(k.aLevels[nLevel].im, irLevelPos0, irLevelPos1, mnPatchSize / 2 + 1)) {
    mbTemplateBad = true;
    return;
  }
  mbTemplateBad = false;

  copy(k.aLevels[nLevel].im, mimTemplate, mnPatchSize, mnPatchSize, irLevelPos1 - mirCenter(1), irLevelPos0 - mirCenter(0));
  //copy(k.aLevels[nLevel].im, mimTemplate, mimTemplate.size(), irLevelPos - mirCenter);
  
  MakeTemplateSums();
}

// Convenient wrapper for the above
void PatchFinder::MakeTemplateCoarseNoWarp(MapPoint &p)
{
  MakeTemplateCoarseNoWarp(*p.pPatchSourceKF, p.nSourceLevel, p.irCenter(0), p.irCenter(1));
}

// Finds the sum, and sum-squared, of template pixels. These sums are used
// to calculate the ZMSSD.
inline void PatchFinder::MakeTemplateSums()
{
  int nSum = 0;
  int nSumSq = 0;
  for(int i=0;i<mnPatchSize;i++)
    for(int j=0;j<mnPatchSize;j++) {
      int b = mimTemplate.at<unsigned char>(i, j);
      nSum += b;
      nSumSq +=b * b;
    }
  mnTemplateSum = nSum;
  mnTemplateSumSq = nSumSq;
}

// One of the main functions of the class! Looks at the appropriate level of 
// the target keyframe to try and find the template. Looks only at FAST corner points
// which are within radius nRange of the center. (Params are supplied in Level0
// coords.) Returns true on patch found.
bool PatchFinder::FindPatchCoarse(Eigen::Vector2d irPos, KeyFrame &kf, unsigned int nRange)
{
  mbFound = false;
  
  // Convert from L0 coords to search level quantities
  int nLevelScale = LevelScale(mnSearchLevel);
  irPos = irPos / nLevelScale;
  nRange = (nRange + nLevelScale - 1) / nLevelScale;
  
  // Bounding box of search circle
  int nTop = irPos(1) - nRange;
  int nBottomPlusOne = irPos(1) + nRange + 1;
  int nLeft = irPos(0) - nRange;
  int nRight = irPos(0) + nRange;
  
  // Ref variable for the search level
  Level &L = kf.aLevels[mnSearchLevel];
  
  // Some bounds checks on the bounding box..
  if(nTop < 0)
    nTop = 0;

  if(nTop >= L.im.rows)
    return false;
  if(nBottomPlusOne <= 0)
    return false;
  
  // The next section finds all the FAST corners in the target level which 
  // are near enough the search center. It's a bit optimised to use 
  // a corner row look-up-table, since otherwise the routine
  // would spend a long time trawling throught the whole list of FAST corners!
  vector<Eigen::Vector2d>::iterator i;
  vector<Eigen::Vector2d>::iterator i_end;
  
  i = L.vCorners.begin() + L.vCornerRowLUT[nTop];
  
  if(nBottomPlusOne >= L.im.rows)
    i_end = L.vCorners.end();
  else 
    i_end = L.vCorners.begin() + L.vCornerRowLUT[nBottomPlusOne];
  

  Eigen::Vector2d irBest(-1,-1);  // Best match so far
  int nBestSSD = mnMaxSSD + 1;    // Best score so far is beyond the max allowed
  
  for(; i<i_end; i++) {         // For each corner ...                     
    if( (*i)(0) < nLeft || (*i)(0) > nRight)
      continue;
    if((irPos - *i).squaredNorm() > nRange * nRange)
	    continue;              // ... reject all those not close enough..

    int nSSD;                // .. and find the ZMSSD at those near enough.
    nSSD = ZMSSDAtPoint(L.im, (int) (*i)(0), (int)(*i)(1));
    if(nSSD < nBestSSD) {     // Best yet?
	    irBest = *i;
	    nBestSSD = nSSD;
	  }
  } // done looping over corners
  
  if(nBestSSD < mnMaxSSD) {     // Found a valid match?
    mv2CoarsePos= LevelZeroPos(irBest, mnSearchLevel);
    mbFound = true;
  } else
    mbFound = false;
  return mbFound;
}

// Makes an inverse composition template out of the coarse template.
// Includes calculating image of derivatives (gradients.) The inverse composition
// used here operates on three variables: x offet, y offset, and difference in patch
// means; hence things like mm3HInv are dim 3, but the trivial mean jacobian 
// (always unity, for each pixel) is not stored.
void PatchFinder::MakeSubPixTemplate() {
  mimJacs[0].resize(mnPatchSize - 2,mnPatchSize - 2);
  mimJacs[1].resize(mnPatchSize - 2,mnPatchSize - 2);
  Eigen::Matrix3d m3H = Eigen::Matrix3d::Zero(); // This stores jTj.

  Eigen::Vector2i ir;
  for(ir(0) = 1; ir(0) < mnPatchSize - 1; ir(0)++)
    for(ir(1) = 1; ir(1) < mnPatchSize - 1; ir(1)++) {
        Eigen::Vector2d v2Grad;

        v2Grad(0) = 0.5 * (mimTemplate.at<unsigned char>(ir(1), ir(0) + 1) - mimTemplate.at<unsigned char>(ir(1), ir(0) - 1));
        v2Grad(1) = 0.5 * (mimTemplate.at<unsigned char>(ir(1) + 1, ir(0)) - mimTemplate.at<unsigned char>(ir(1) - 1, ir(0)));
        mimJacs[0](ir(0)-1, ir(1)-1) = v2Grad(0);
        mimJacs[1](ir(0)-1, ir(1)-1) = v2Grad(1);
        Eigen::Vector3d v3Grad = myPatchFinderUnproject(v2Grad); // This adds the mean-difference jacobian..
        m3H+= v3Grad * v3Grad.transpose();// Populate JTJ.
    }
  
  // Invert JTJ..
//  Cholesky<3> chol(m3H);
//  mm3HInv = chol.get_inverse();
  mm3HInv = m3H.inverse();
  
  mv2SubPixPos = mv2CoarsePos; // Start the sub-pixel search at the result of the coarse search..
  mdMeanDiff = 0.0;
}

// Iterate inverse composition until convergence. Since it should never have 
// to travel more than a pixel's distance, set a max number of iterations; 
// if this is exceeded, consider the IC to have failed.
bool PatchFinder::IterateSubPixToConvergence(KeyFrame &kf, int nMaxIts)
{
  const double dConvLimit = 0.03;
  bool bConverged = false;
  int nIts;
  for(nIts = 0; nIts < nMaxIts && !bConverged; nIts++){
      double dUpdateSquared = IterateSubPix(kf);
      if(dUpdateSquared < 0) // went off edge of image
        return false;
      if(dUpdateSquared < dConvLimit*dConvLimit)
        return true;
  }
  return false;
}

// Single iteration of inverse composition. This compares integral image positions in the 
// template image to floating point positions in the target keyframe. Interpolation is
// bilinear, and performed manually since this is a special case where the mixing
// fractions for each pixel are identical.
double PatchFinder::IterateSubPix(KeyFrame &kf) {
  Eigen::Vector2i ir;

  // Search level pos of patch center
  Eigen::Vector2d  v2Center;
  v2Center= LevelNPos(mv2SubPixPos, mnSearchLevel);

  int x_border = (v2Center[0] > 0.0 ? v2Center[0] + 0.5 : v2Center[0] - 0.5);
  int y_border = (v2Center[1] > 0.0 ? v2Center[1] + 0.5 : v2Center[1] - 0.5);

  if(!in_image_with_border(kf.aLevels[mnSearchLevel].im, x_border, y_border, mnPatchSize / 2 + 1))
    return -1.0;       // Negative return value indicates off edge of image 

  // Position of top-left corner of patch in search level
  Eigen::Vector2d v2Base;
  v2Base(0) = v2Center[0] - mirCenter(0);
  v2Base(1) = v2Center[1] - mirCenter(1);
  
  // I.C. JT*d accumulator
  Eigen::Vector3d v3Accum = Eigen::Vector3d::Zero();

  unsigned char* pTopLeftPixel;
  
  // Each template pixel will be compared to an interpolated target pixel
  // The target value is made using bilinear interpolation as the weighted sum
  // of four target image pixels. Calculate mixing fractions:
  double dX = v2Base(0)-floor(v2Base(0)); // Distances from pixel center of TL pixel
  double dY = v2Base(1)-floor(v2Base(1));
  float fMixTL = (1.0 - dX) * (1.0 - dY);
  float fMixTR = (dX)       * (1.0 - dY);
  float fMixBL = (1.0 - dX) * (dY);
  float fMixBR = (dX)       * (dY);
  
  // Loop over template image
  unsigned long nRowOffset = &kf.aLevels[mnSearchLevel].im.at<unsigned char>(1,0) - &kf.aLevels[mnSearchLevel].im.at<unsigned char>(0,0);
  for(ir(1) = 1; ir(1) < mnPatchSize - 1; ir(1)++) {
    pTopLeftPixel = &kf.aLevels[mnSearchLevel].im.at<unsigned char>((int) v2Base(1) + ir(1), (int) v2Base(0) + 1); // n.b. the x=1 offset, as with y
    for(ir(0) = 1; ir(0) < mnPatchSize - 1; ir(0)++)	{
      float fPixel =   // Calc target interpolated pixel
        fMixTL * pTopLeftPixel[0]          + fMixTR * pTopLeftPixel[1] + 
        fMixBL * pTopLeftPixel[nRowOffset] + fMixBR * pTopLeftPixel[nRowOffset + 1];
      pTopLeftPixel++;
      double dDiff = fPixel - mimTemplate.at<unsigned char>(ir(1), ir(0)) + mdMeanDiff;

      v3Accum(0) += dDiff * mimJacs[0](ir(0)-1,ir(1)-1);
      v3Accum(1) += dDiff * mimJacs[1](ir(0)-1,ir(1)-1);
      v3Accum(2) += dDiff;  // Update JT*d

    }
  }
  
  // All done looping over image - find JTJ^-1 * JTd:
  Eigen::Vector3d v3Update = mm3HInv * v3Accum;

  mv2SubPixPos -= v3Update.segment(0,2) * LevelScale(mnSearchLevel);
  mdMeanDiff -= v3Update[2];
  
  double dPixelUpdateSquared = v3Update.segment(0,2).dot(v3Update.segment(0,2));
  return dPixelUpdateSquared;
}

int PatchFinder::ZMSSDAtPoint(cv::Mat& img, int icol, int irow) {
  if(!in_image_with_border(img, icol, irow, (int)mirCenter(0)))
    return mnMaxSSD + 1;
  
  Eigen::Vector2i irImgBase(icol - (int)mirCenter(0), irow - (int)mirCenter(1));
  unsigned char *imagepointer;
  unsigned char *templatepointer;
  
  int nImageSumSq = 0;
  int nImageSum = 0;
  int nCrossSum = 0;

  for(int nRow = 0; nRow < mnPatchSize; nRow++)	{
    imagepointer = img.ptr<unsigned char>(irImgBase(1) + nRow);
    templatepointer = mimTemplate.ptr<unsigned char>(nRow);
    for(int nCol = 0; nCol < mnPatchSize; nCol++){
        int n = imagepointer[irImgBase(0) + nCol];
	      nImageSum += n;
	      nImageSumSq += n*n;
        nCrossSum += n * templatepointer[nCol];
      }
	}
  
  int SA = mnTemplateSum;
  int SB = nImageSum;
  
  int N = mnPatchSize * mnPatchSize;
  return ((2*SA*SB - SA*SA - SB*SB)/N + nImageSumSq + mnTemplateSumSq - 2*nCrossSum);
}



