#include "SmallBlurryImage.h"

using namespace std;

Eigen::Vector2d SmallBlurryImage::mirSize(-1,-1);

SmallBlurryImage::SmallBlurryImage(KeyFrame &kf, double dBlur) {
  mbMadeJacs = false;
  MakeFromKF(kf, dBlur);
}

SmallBlurryImage::SmallBlurryImage() {
  mbMadeJacs = false;
}

// Make a SmallBlurryImage from a KeyFrame This fills in the mimSmall
// image (Which is just a small un-blurred version of the KF) and
// mimTemplate (which is a floating-point, zero-mean blurred version
// of the above)
void SmallBlurryImage::MakeFromKF(KeyFrame &kf, double dBlur)
{
  if(mirSize(0) == -1) {
    mirSize(0) = kf.aLevels[3].im.cols / 2;
    mirSize(1) = kf.aLevels[3].im.rows / 2;
  }
  mbMadeJacs = false;

  mimSmall.create((int)mirSize(1), (int)mirSize(0), CV_8UC1);
  mimTemplate.create((int)mirSize(1), (int)mirSize(0), CV_32FC1);
  cv::resize(kf.aLevels[3].im, mimSmall, mimSmall.size());

  mbMadeJacs = false;

  int indice = 0;
  unsigned int nSum = 0;

  for(indice = 0; indice < mimSmall.rows*mimSmall.cols; indice++){
    nSum+= (unsigned int)mimSmall.data[indice];
  }

  float fMean = ((float) nSum)/(mimSmall.rows*mimSmall.cols);

  for(int x = 0; x < mimTemplate.cols; x++){
      for(int y = 0; y < mimTemplate.rows; y++){
          mimTemplate.at<float>(y, x) = mimSmall.data[y*mimSmall.step + x*mimSmall.channels()] - fMean;
      }
  }

  //convolveGaussian(mimTemplate, dBlur);

  if(dBlur <= 2.0)
    cv::GaussianBlur(mimTemplate, mimTemplate, cv::Size(9, 9), dBlur, dBlur, cv::BORDER_REPLICATE);
  else
    cv::GaussianBlur(mimTemplate, mimTemplate, cv::Size(17, 17), dBlur, dBlur, cv::BORDER_REPLICATE); /*TODO: try to optimize*/
}

// Make the jacobians (actually, no more than a gradient image) of the blurred template
void SmallBlurryImage::MakeJacs() {

  mimImageJacs.create((int)mirSize(1), (int)mirSize(0), CV_32FC2);
  // Fill in the gradient image
  for(int x = 0; x < mimImageJacs.cols; x++){
      for(int y = 0; y < mimImageJacs.rows; y++){

          if(in_image_with_border(mimTemplate, x, y, 1)) {
              mimImageJacs.at<cv::Vec2f>(y, x)[0] = mimTemplate.at<float>(y, x+1) - mimTemplate.at<float>(y, x-1);
              mimImageJacs.at<cv::Vec2f>(y, x)[1] = mimTemplate.at<float>(y+1, x) - mimTemplate.at<float>(y-1, x);

          }else{
              mimImageJacs.at<cv::Vec2f>(y, x)[0] = 0.0;
              mimImageJacs.at<cv::Vec2f>(y, x)[1] = 0.0;
          }

      }
  }

  mbMadeJacs = true;
}

// Calculate the zero-mean SSD between one image and the next.
// Since both are zero mean already, just calculate the SSD...
double SmallBlurryImage::ZMSSD(SmallBlurryImage &other)
{
  double dSSD = 0.0;

  for(int x = 0; x< mimTemplate.cols; x++){
    for(int y = 0; y< mimTemplate.rows; y++){
      double dDiff = mimTemplate.at<float>(y, x) - other.mimTemplate.at<float>(y, x);
      dSSD += dDiff * dDiff;
    }
  }

  return dSSD;
}


// Find an SE2 which best aligns an SBI to a target
// Do this by ESM-tracking a la Benhimane & Malis
pair<mySE2,double> SmallBlurryImage::IteratePosRelToTarget(SmallBlurryImage &other, int nIterations) {

  mySE2 se2CtoC;
  mySE2 se2WfromC;

  Eigen::Vector2d irCenter = mirSize / 2;

  Eigen::Vector2d vtrans;
  vtrans[0] = irCenter(0);
  vtrans[1] = irCenter(1);

  se2WfromC.get_translation() = vtrans;

  pair<mySE2, double> result_pair_eigen;
  if(!other.mbMadeJacs) {
    cerr << "You spanner, you didn't make the jacs for the target." << endl;
    assert(other.mbMadeJacs);
  }

  double dMeanOffset = 0.0;
  Eigen::Vector4d v4Accum;

  Eigen::VectorXd v10Triangle(10);

  cv::Mat imWarped;
  imWarped.create((int)mirSize(1), (int)mirSize(0), CV_32FC1);

  double dFinalScore = 0.0;
  for(int it = 0; it<nIterations; it++) {
    dFinalScore = 0.0;
    v4Accum.setZero();
    v10Triangle.setZero();
    Eigen::Vector4d v4Jac;

    mySE2 se2XForm = se2WfromC * se2CtoC * se2WfromC.inverse();


    // Make the warped current image template:
    Eigen::Vector2d v2Zero(0,0);
    Eigen::Vector2d trasm(se2XForm.get_translation()[0], se2XForm.get_translation()[1]);
    Eigen::Matrix2d rotm;
    rotm(0,0) = se2XForm.get_rotation().get_matrix()(0, 0);
    rotm(1,0) = se2XForm.get_rotation().get_matrix()(1, 0);
    rotm(0,1) = se2XForm.get_rotation().get_matrix()(0, 1);
    rotm(1,1) = se2XForm.get_rotation().get_matrix()(1, 1);
    transform_image(mimTemplate, imWarped, rotm, trasm, v2Zero, -9e20f);   

    // Now compare images, calc differences, and current image jacobian:
    for(int i=0;i<mirSize(0);i++)
      for(int j=0;j<mirSize(1);j++) {
        if(!in_image_with_border(imWarped, i, j, 1))
          continue;

          float l,r,u,d,here;
          l = imWarped.at<float>(j, i-1);
          r = imWarped.at<float>(j, i+1);
          u = imWarped.at<float>(j-1, i);
          d = imWarped.at<float>(j+1, i);
          here = imWarped.at<float>(j, i);
          if(l + r + u + d + here < -9999.9)   // This means it's out of the image; c.f. the -9e20f param to transform.
            continue;

          Eigen::Vector2d v2CurrentGrad;
          v2CurrentGrad(0) = r - l; // Missing 0.5 factor
          v2CurrentGrad(1) = d - u;

          Eigen::Vector2d v2SumGrad;
          v2SumGrad(0) = 0.25 * (v2CurrentGrad(0)  + other.mimImageJacs.at<cv::Vec2f>(j, i)[0]);
          v2SumGrad(1) = 0.25 * (v2CurrentGrad(1)  + other.mimImageJacs.at<cv::Vec2f>(j, i)[1]);

          // Why 0.25? This is from missing 0.5 factors: One for
          // the fact we average two gradients, the other from
          // each gradient missing a 0.5 factor.

          v4Jac(0) = v2SumGrad(0);
          v4Jac(1) = v2SumGrad(1);
          v4Jac(2) = -((double)j - irCenter(1)) * v2SumGrad(0) + ((double)i - irCenter(0)) * v2SumGrad(1);
          v4Jac(3) = 1.0;

          double dDiff = imWarped.at<float>(j, i) - other.mimTemplate.at<float>(j, i) + dMeanOffset;
          dFinalScore += dDiff * dDiff;

          v4Accum += dDiff * v4Jac;

          // Speedy fill of the LL triangle of JTJ:
          v10Triangle(0)  += v4Jac(0) * v4Jac(0);
          v10Triangle(1)  += v4Jac(1) * v4Jac(0);
          v10Triangle(2)  += v4Jac(1) * v4Jac(1);
          v10Triangle(3)  += v4Jac(2) * v4Jac(0);
          v10Triangle(4)  += v4Jac(2) * v4Jac(1);
          v10Triangle(5)  += v4Jac(2) * v4Jac(2);
          v10Triangle(6)  += v4Jac(0);
          v10Triangle(7)  += v4Jac(1);
          v10Triangle(8)  += v4Jac(2);
          v10Triangle(9)  += 1.0;
      }

    Eigen::VectorXd v4Update;

    // Solve for JTJ-1JTv;
    {
      Eigen::Matrix4d m4;
      int v=0;
      for(int j=0; j<4; j++)
        for(int i=0; i<=j; i++) {
            m4(j,i) = m4(i,j) = v10Triangle(v++);
        }

      v4Update = m4.inverse()*v4Accum;

    }
    mySE2 se2Update;
    se2Update.get_translation() = -v4Update.segment(0,2);
    se2Update.get_rotation() = mySO2::exp(-v4Update(2));

    se2CtoC = se2CtoC * se2Update;
    dMeanOffset -= v4Update(3);
  }

  result_pair_eigen.first = se2CtoC;
  result_pair_eigen.second = dFinalScore;

  return result_pair_eigen;
}

Eigen::Vector3d myUnproject_Blur(Eigen::Vector2d v)
{
    Eigen::Vector3d result;
    result(0) = v(0);
    result(1) = v(1);
    result(2) = 1.0;

    return result;
}

Eigen::Vector2d myProject_Blur(Eigen::Vector3d& v)
{
    Eigen::Vector2d result;
    result(0) = v(0)/v(2);
    result(1) = v(1)/v(2);

    return result;
}

// What is the 3D camera rotation (zero trans) mySE3 which causes an
// input image SO2 rotation?
mySE3 SmallBlurryImage::SE3fromSE2(mySE2 se2, ATANCamera camera) {
  // Do this by projecting two points, and then iterating the mySE3 (SO3
  // actually) until convergence. It might seem stupid doing this so
  // precisely when the whole SE2-finding is one big hack, but hey.

  camera.SetImageSize(mirSize);

  Eigen::Vector2d av2Turned[2];   // Our two warped points in pixels
  Eigen::Vector2d vtmp1;
  vtmp1(0) = mirSize(0) / 2;
  vtmp1(1) = mirSize(1) / 2;
  Eigen::Vector2d vtmp2;
  vtmp2(0) = 5;
  vtmp2(1) = 0;
  Eigen::Vector2d vtmp3;
  vtmp3(0) = -5;
  vtmp3(1) = 0;
  av2Turned[0] = vtmp1 + se2 * vtmp2;
  av2Turned[1] = vtmp1 + se2 * vtmp3;

  Eigen::Vector2d av2Turned_Eigen[2];
  av2Turned_Eigen[0](0) = av2Turned[0][0];
  av2Turned_Eigen[0](1) = av2Turned[0][1];
  av2Turned_Eigen[1](0) = av2Turned[1][0];
  av2Turned_Eigen[1](1) = av2Turned[1][1];

  Eigen::Vector2d vtmp1_eigen(vtmp1[0], vtmp1[1]);
  Eigen::Vector2d vtmp2_eigen(vtmp2[0], vtmp2[1]);;
  Eigen::Vector2d vtmp3_eigen(vtmp3[0], vtmp3[1]);;

  Eigen::Vector2d aux1 = vtmp1_eigen + vtmp2_eigen;
  Eigen::Vector2d aux2 = vtmp1_eigen + vtmp3_eigen;

  Eigen::Vector3d av3OrigPoints[2];   // 3D versions of these points.
  av3OrigPoints[0] = myUnproject_Blur(camera.UnProject(aux1));
  av3OrigPoints[1] = myUnproject_Blur(camera.UnProject(aux2));

  mySO3 so3;

  for(int it = 0; it<3; it++) {
    myWLS<3> wls;  // lazy; no need for the 'W'

    wls.add_prior(10.0);
    for(int i=0; i<2; i++) {
      // Project into the image to find error
      Eigen::Vector3d v3Cam_Eigen = so3 * av3OrigPoints[i];

      Eigen::Vector2d v2Implane_Eigen = myProject_Blur(v3Cam_Eigen);


      Eigen::Vector2d v2Pixels_Eigen = camera.Project(v2Implane_Eigen);
      Eigen::Vector2d v2Error = av2Turned_Eigen[i] - v2Pixels_Eigen;

      Eigen::Matrix2d m2CamDerivs_Eigen = camera.GetProjectionDerivs_Eigen();

      Eigen::MatrixXd m23Jacobian_Eigen(2, 3);

      double dOneOverCameraZ = 1.0 / v3Cam_Eigen(2);
      for(int m=0; m<3; m++) {
        const Eigen::Vector3d v3Motion_Eigen = mySO3::generator_field(m, v3Cam_Eigen);

        Eigen::Vector2d v2CamFrameMotion_Eigen;
        v2CamFrameMotion_Eigen(0) = (v3Motion_Eigen(0) - v3Cam_Eigen(0) * v3Motion_Eigen(2) * dOneOverCameraZ)* dOneOverCameraZ;
        v2CamFrameMotion_Eigen(1) = (v3Motion_Eigen(1) - v3Cam_Eigen(1) * v3Motion_Eigen(2) * dOneOverCameraZ)* dOneOverCameraZ;

        Eigen::Vector2d aux = m2CamDerivs_Eigen* v2CamFrameMotion_Eigen;
        m23Jacobian_Eigen(0, m) = aux(0);
        m23Jacobian_Eigen(1, m) = aux(1);
      }

      Eigen::MatrixXd jac1, jac2;
      jac1 = m23Jacobian_Eigen.block(0,0,1,3);
      jac2 = m23Jacobian_Eigen.block(1,0,1,3);

      wls.add_mJ(v2Error(0), jac1, 1.0);
      wls.add_mJ(v2Error(1), jac2, 1.0);
    }

    wls.compute();

    Eigen::Vector3d v3Res = wls.get_mu();

    so3 = mySO3::exp(v3Res) * so3;
  }

  mySE3 se3Result;
  se3Result.get_rotation() = so3;
  return se3Result;
}
