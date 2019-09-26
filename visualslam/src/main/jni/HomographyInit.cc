#include "HomographyInit.h"

using namespace std;

Eigen::Vector3d myUnproject(Eigen::Vector2d& v) {
  Eigen::Vector3d result;
  result(0) = v(0);
  result(1) = v(1);
  result(2) = 1.0;

  return result;
}

Eigen::Vector2d myProject(Eigen::Vector3d& v) {
  Eigen::Vector2d result;
  result(0) = v(0)/v(2);
  result(1) = v(1)/v(2);

  return result;
}

bool HomographyInit::IsHomographyInlier(Eigen::Matrix3d &m3Homography, HomographyMatch match) {
  Eigen::Vector3d v = m3Homography*myUnproject(match.v2CamPlaneFirst);
  Eigen::Vector2d v2Projected =  myProject(v);
  Eigen::Vector2d v2Error =  match.v2CamPlaneSecond - v2Projected;
  Eigen::Vector2d v2PixelError = match.m2PixelProjectionJac * v2Error;
  double dSquaredError =  v2PixelError.dot(v2PixelError);
  return (dSquaredError < mdMaxPixelErrorSquared);
}

double HomographyInit::MLESACScore(Eigen::Matrix3d &m3Homography, HomographyMatch match) {
  Eigen::Vector3d v = m3Homography * myUnproject(match.v2CamPlaneFirst);
  Eigen::Vector2d v2Projected = myProject(v);
  Eigen::Vector2d v2Error = match.v2CamPlaneSecond - v2Projected;
  Eigen::Vector2d v2PixelError =  match.m2PixelProjectionJac * v2Error;
  double dSquaredError =  v2PixelError.dot(v2PixelError);
  if(dSquaredError > mdMaxPixelErrorSquared)
    return mdMaxPixelErrorSquared;
  else
    return dSquaredError;
}

bool HomographyInit::Compute(vector<HomographyMatch> vMatches, double dMaxPixelError, mySE3 &se3SecondCameraPose) {
  mdMaxPixelErrorSquared = dMaxPixelError * dMaxPixelError;
  mvMatches = vMatches;
  
  // Find best homography from minimal sets of image matches
  BestHomographyFromMatches_MLESAC();
  
  // Generate the inlier set, and refine the best estimate using this
  mvHomographyInliers.clear();
  for(unsigned int i=0; i<mvMatches.size(); i++)
    if(IsHomographyInlier(mm3BestHomography, mvMatches[i]))
      mvHomographyInliers.push_back(mvMatches[i]);

  for(int iteration = 0; iteration < 5; iteration++)
    RefineHomographyWithInliers();
  
  // Decompose the best homography into a set of possible decompositions
  DecomposeHomography();

  // At this stage should have eight decomposition options, if all went according to plan
  if(mvDecompositions.size() != 8)
    return false;
  
  // And choose the best one based on visibility constraints
  ChooseBestDecomposition();
  
  se3SecondCameraPose = mvDecompositions[0].se3SecondFromFirst;
  return true;
}

Eigen::Matrix3d HomographyInit::HomographyFromMatches(vector<HomographyMatch> vMatches) {
  unsigned int nPoints = vMatches.size();
  assert(nPoints >= 4);
  int nRows = 2*nPoints;
  if(nRows < 9)
    nRows = 9;

  Eigen::MatrixXd m2Nx9(nRows, 9);

  for(unsigned int n=0; n<nPoints; n++){
    double u = vMatches[n].v2CamPlaneSecond(0);
    double v = vMatches[n].v2CamPlaneSecond(1);
    
    double x = vMatches[n].v2CamPlaneFirst(0);
    double y = vMatches[n].v2CamPlaneFirst(1);
    
    // [u v]T = H [x y]T
    m2Nx9(n*2+0, 0) = x;
    m2Nx9(n*2+0, 1) = y;
    m2Nx9(n*2+0, 2) = 1;
    m2Nx9(n*2+0, 3) = 0;
    m2Nx9(n*2+0, 4) = 0;
    m2Nx9(n*2+0, 5) = 0;
    m2Nx9(n*2+0, 6) = -x*u;
    m2Nx9(n*2+0, 7) = -y*u;
    m2Nx9(n*2+0, 8) = -u;

    m2Nx9(n*2+1, 0) = 0;
    m2Nx9(n*2+1, 1) = 0;
    m2Nx9(n*2+1, 2) = 0;
    m2Nx9(n*2+1, 3) = x;
    m2Nx9(n*2+1, 4) = y;
    m2Nx9(n*2+1, 5) = 1;
    m2Nx9(n*2+1, 6) = -x*v;
    m2Nx9(n*2+1, 7) = -y*v;
    m2Nx9(n*2+1, 8) = -v;
  }

  if(nRows == 9)  
   for(int i=0; i<9; i++){  // Zero the last row of the matrix,
     m2Nx9(8,i)= 0.0;
  }

  // The right null-space of the matrix gives the homography...

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(m2Nx9, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd V = svd.matrixV().transpose();

  Eigen::Matrix3d m3Homography = Eigen::Matrix3d::Identity();

  m3Homography.block(0, 0, 1, 3) = V.block(8, 0, 1, 3);
  m3Homography.block(1, 0, 1, 3) = V.block(8, 3, 1, 3);
  m3Homography.block(2, 0, 1, 3) = V.block(8, 6, 1, 3);

  return m3Homography;
}

// Throughout the whole thing,
// SecondView = Homography * FirstView

void HomographyInit::RefineHomographyWithInliers() {
  myWLS<9> wls;
  wls.add_prior(1.0);
  
  vector<double> vdErrorSquared;
  vector<Eigen::MatrixXd> vmJacobians_eigen;
  vector<Eigen::Vector2d> vvErrors;
  vector<double> vdErrorSquared_eigen;
  
  for(unsigned int i=0; i<mvHomographyInliers.size(); i++) {
    // First, find error.
    Eigen::Vector2d v2First = mvHomographyInliers[i].v2CamPlaneFirst;
    Eigen::Vector3d v3Second = mm3BestHomography * myUnproject(v2First);
    Eigen::Vector2d v2Second = myProject(v3Second);
    Eigen::Vector2d v2Second_real = mvHomographyInliers[i].v2CamPlaneSecond;
    Eigen::Vector2d v2Error = mvHomographyInliers[i].m2PixelProjectionJac * (v2Second_real - v2Second);
    
    vdErrorSquared.push_back(v2Error.dot(v2Error));
    vvErrors.push_back(v2Error);

    Eigen::MatrixXd m29Jacobian(2,9);
    Eigen::Vector3d cunproject;
    double dDenominator = v3Second(2);
    double dNumerator;

    // Jacobians wrt to the elements of the homography:
    // For x:
    cunproject = myUnproject(v2First) / dDenominator;
    m29Jacobian(0,0) = cunproject(0);
    m29Jacobian(0,1) = cunproject(1);
    m29Jacobian(0,2) = cunproject(2);
    m29Jacobian.block(0,3,1,3).setZero();
    dNumerator = v3Second(0);
    cunproject = -myUnproject(v2First) * dNumerator / (dDenominator * dDenominator);
    m29Jacobian(0,6) = cunproject(0);
    m29Jacobian(0,7) = cunproject(1);
    m29Jacobian(0,8) = cunproject(2);
    // For y:
    m29Jacobian.block(1,0,1,3).setZero();
    cunproject = myUnproject(v2First) / dDenominator;
    m29Jacobian(1,3) = cunproject(0);
    m29Jacobian(1,4) = cunproject(1);
    m29Jacobian(1,5) = cunproject(2);
    dNumerator = v3Second(1);
    cunproject = -myUnproject(v2First) * dNumerator / (dDenominator * dDenominator);
    m29Jacobian(1,6) = cunproject(0);
    m29Jacobian(1,7) = cunproject(1);
    m29Jacobian(1,8) = cunproject(2);

    vmJacobians_eigen.push_back(mvHomographyInliers[i].m2PixelProjectionJac * m29Jacobian);
  }
  
  // Calculate robust sigma:
  vector<double> vdd = vdErrorSquared;
//  std::cout << "vdErrorSquared: "<< vdErrorSquared.size()<< std::endl;
  double dSigmaSquared = Tukey::FindSigmaSquared(vdd);
  
  // Add re-weighted measurements to WLS:
  for(unsigned int i=0; i<mvHomographyInliers.size(); i++) {
    double dWeight = Tukey::Weight(vdErrorSquared[i], dSigmaSquared);
    Eigen::MatrixXd jac1, jac2;
    jac1 = vmJacobians_eigen[i].block(0,0,1,9);
    jac2 = vmJacobians_eigen[i].block(1,0,1,9);
    wls.add_mJ((int)vvErrors[i](0), jac1, dWeight);
    wls.add_mJ((int)vvErrors[i](1), jac2, dWeight);
  }
  wls.compute();
  Eigen::VectorXd v9Update = wls.get_mu();

  Eigen::Matrix3d m3Update;
  m3Update(0,0) = v9Update(0);
  m3Update(0,1) = v9Update(1);
  m3Update(0,2) = v9Update(2);
  m3Update(1,0) = v9Update(3);
  m3Update(1,1) = v9Update(4);
  m3Update(1,2) = v9Update(5);
  m3Update(2,0) = v9Update(6);
  m3Update(2,1) = v9Update(7);
  m3Update(2,2) = v9Update(8);

  mm3BestHomography += m3Update;
}

void HomographyInit::BestHomographyFromMatches_MLESAC() {
  // Not many matches? Don't do ransac, throw them all in a pot and see what comes out.
  if(mvMatches.size() < 10) {
    mm3BestHomography = HomographyFromMatches(mvMatches);
    return;
  }
  
  // Enough matches? Run MLESAC.
  int anIndices[4];
  
  mm3BestHomography.setIdentity();
  double dBestError = 999999999999999999.9;
  
  // Do 300 MLESAC trials.
  for(int nR = 0; nR < 300 ; nR++) { 
    // Find set of four unique matches
    for(int i=0; i<4; i++) {
	    bool isUnique = false;
	    int n;
	    while(!isUnique) {
        n = rand() % mvMatches.size();
        isUnique =true;
        for(int j=0; j<i && isUnique; j++)
	        if(anIndices[j] == n)
	          isUnique = false;
	    }
	    anIndices[i] = n;
	  }
    vector<HomographyMatch> vMinimalMatches;
    for(int i=0; i<4; i++)
      vMinimalMatches.push_back(mvMatches[anIndices[i]]);
      
    // Find a homography from the minimal set..
    Eigen::Matrix3d m3Homography_Eigen = HomographyFromMatches(vMinimalMatches);
    
    //..and sum resulting MLESAC score
    double dError = 0.0;
    for(unsigned int i=0; i<mvMatches.size(); i++) {
      dError += MLESACScore(m3Homography_Eigen, mvMatches[i]);
    }
    
    if(dError < dBestError)	{
      mm3BestHomography = m3Homography_Eigen;
      dBestError = dError;
    }
  }
}

void HomographyInit::DecomposeHomography()
{
  mvDecompositions.clear();

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(mm3BestHomography, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d v3Diag = svd.singularValues();

  double d1 = fabs((double)v3Diag(0)); // The paper suggests the square of these (e.g. the evalues of AAT)
  double d2 = fabs((double)v3Diag(1)); // should be used, but this is wrong. c.f. Faugeras' book.
  double d3 = fabs((double)v3Diag(2));

  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  double s = U.determinant() * V.determinant();
  double dPrime_PM = d2;
  
  int nCase;
  if(d1 != d2 && d2 != d3)
    nCase = 1;
  else if( d1 == d2 && d2 == d3)
    nCase = 3;
  else
    nCase = 2;
  
  if(nCase != 1){
//    std::cout << "  Homographyinit: This motion case is not implemented or is degenerate. Try again. " << endl;
    return;
  }
  
  double x1_PM;
  double x2;
  double x3_PM;

  // All below deals with the case = 1 case.
  // Case 1 implies (d1 != d3) 
  { // Eq. 12
    x1_PM = sqrt((d1*d1 - d2*d2) / (d1*d1 - d3*d3));
    x2    = 0;
    x3_PM = sqrt((d2*d2 - d3*d3) / (d1*d1 - d3*d3));
  };
  
  double e1[4] = {1.0,-1.0, 1.0, -1.0};
  double e3[4] = {1.0, 1.0, -1.0,-1.0};

  double e1_Eigen[4] = {1.0,-1.0, 1.0, -1.0};
  double e3_Eigen[4] = {1.0, 1.0, -1.0,-1.0};

  Eigen::Vector3d v3np;
  HomographyDecomposition decomposition;

  // Case 1, d' > 0:
  decomposition.d = s * dPrime_PM;
  for(int signs=0; signs<4; signs++) {
    // Eq 13
    decomposition.m3Rp = Eigen::Matrix3d::Identity();

    double dSinTheta = (d1 - d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
    double dCosTheta = (d1 * x3_PM * x3_PM + d3 * x1_PM * x1_PM) / d2;
    decomposition.m3Rp(0, 0) = dCosTheta;
    decomposition.m3Rp(0, 2) = -dSinTheta;
    decomposition.m3Rp(2, 0) = dSinTheta;
    decomposition.m3Rp(2, 2) = dCosTheta;
    
    // Eq 14
    decomposition.v3Tp(0) = (d1 - d3) * x1_PM * e1_Eigen[signs];
    decomposition.v3Tp(1) = 0.0;
    decomposition.v3Tp(2) = (d1 - d3) * -x3_PM * e3_Eigen[signs];

    v3np(0) = x1_PM * e1[signs];
    v3np(1) = x2;
    v3np(2) = x3_PM * e3[signs];

    decomposition.v3n = V * v3np;

    mvDecompositions.push_back(decomposition);
  }

  // Case 1, d' < 0:
  decomposition.d = s * -dPrime_PM;
  for(int signs=0; signs<4; signs++){
    // Eq 15
    decomposition.m3Rp= -1 * Eigen::Matrix3d::Identity();

    double dSinPhi = (d1 + d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
    double dCosPhi = (d3 * x1_PM * x1_PM - d1 * x3_PM * x3_PM) / d2;  
    decomposition.m3Rp(0, 0) = dCosPhi;
    decomposition.m3Rp(0, 2) = dSinPhi;
    decomposition.m3Rp(2, 0) = dSinPhi;
    decomposition.m3Rp(2, 2) = -dCosPhi;

    // Eq 16
    decomposition.v3Tp(0) = (d1 + d3) * x1_PM * e1_Eigen[signs];
    decomposition.v3Tp(1) = 0.0;
    decomposition.v3Tp(2) = (d1 + d3) * x3_PM * e3_Eigen[signs];

    v3np(0) = x1_PM * e1[signs];
    v3np(1) = x2;
    v3np(2) = x3_PM * e3[signs];

    decomposition.v3n = V * v3np;

    mvDecompositions.push_back(decomposition);
  }

  // While we have the SVD results calculated here, store the decomposition R and t results as well..
  for(unsigned int i=0; i<mvDecompositions.size(); i++){
    Eigen::Matrix3d rotation = s * U * mvDecompositions[i].m3Rp * V.transpose();
    Eigen::Vector3d translation = U * mvDecompositions[i].v3Tp;

    mvDecompositions[i].se3SecondFromFirst.get_rotation() = rotation;
    mvDecompositions[i].se3SecondFromFirst.get_translation() = translation;
  }
}

bool operator<(const HomographyDecomposition lhs, const HomographyDecomposition rhs) {
  return lhs.nScore < rhs.nScore;
}

static double SampsonusError(Eigen::Vector2d &v2Dash, const Eigen::Matrix3d &m3Essential, Eigen::Vector2d &v2) {
  Eigen::Vector3d v3Dash =myUnproject(v2Dash);
  Eigen::Vector3d v3 = myUnproject(v2);

  Eigen::Vector3d aux = (m3Essential*v3);

  double dError =  aux.dot(v3Dash);

  Eigen::Vector3d fv3 =m3Essential * v3;
  Eigen::Vector3d fTv3Dash = m3Essential.transpose() * v3Dash;

  Eigen::Vector2d fv3Slice;
  fv3Slice(0) = fv3(0);
  fv3Slice(1) = fv3(1);

  Eigen::Vector2d fTv3DashSlice;
  fTv3DashSlice(0) = fTv3Dash(0);
  fTv3DashSlice(1) = fTv3Dash(1);

  return (dError * dError / (fv3Slice.dot(fv3Slice) + fTv3DashSlice.dot(fTv3DashSlice)));
}

void HomographyInit::ChooseBestDecomposition() {

  assert(mvDecompositions.size() == 8);

  for(unsigned int i=0; i<mvDecompositions.size(); i++) {
    HomographyDecomposition &decom = mvDecompositions[i];
    int nPositive = 0;
    for(unsigned int m=0; m<mvHomographyInliers.size(); m++) {
      Eigen::Vector2d v2;
      v2(0) = mvHomographyInliers[m].v2CamPlaneFirst(0);
      v2(1) = mvHomographyInliers[m].v2CamPlaneFirst(1);
      double dVisibilityTest = (mm3BestHomography(2, 0) * v2(0) + mm3BestHomography(2, 1) * v2(1) + mm3BestHomography(2, 2)) / decom.d;
      if(dVisibilityTest > 0.0)
        nPositive++;
    };
    decom.nScore = -nPositive;
  }
  
  sort(mvDecompositions.begin(), mvDecompositions.end());
  mvDecompositions.resize(4);
  
  for(unsigned int i=0; i<mvDecompositions.size(); i++){
    HomographyDecomposition &decom = mvDecompositions[i];
    int nPositive = 0;
    for(unsigned int m=0; m<mvHomographyInliers.size(); m++){

        Eigen::Vector2d v2;
        v2(0) = mvHomographyInliers[m].v2CamPlaneFirst(0);
        v2(1) = mvHomographyInliers[m].v2CamPlaneFirst(1);
        Eigen::Vector3d v3 = myUnproject(v2);

        Eigen::Vector3d v3n = myUnproject(v2);
        v3n(0) = decom.v3n[0];
        v3n(1) = decom.v3n[1];
        v3n(2) = decom.v3n[2];

        double dVisibilityTest = v3.dot(v3n) / decom.d;
        if(dVisibilityTest > 0.0)
          nPositive++;
    }
    decom.nScore = -nPositive;
  }
  
  sort(mvDecompositions.begin(), mvDecompositions.end());
  mvDecompositions.resize(2);
  
  // According to Faugeras and Lustman, ambiguity exists if the two scores are equal
  // but in practive, better to look at the ratio!
  double dRatio = (double) mvDecompositions[1].nScore / (double) mvDecompositions[0].nScore;

//  std::cout <<"dRatio: " << dRatio << std::endl;
//  std::cout <<"mvDecompositions " << mvDecompositions.size()<< std::endl;

  if(dRatio < 0.9) { // no ambiguity!
    mvDecompositions.erase(mvDecompositions.begin() + 1);
  } else {             // two-way ambiguity. Resolve by sampsonus score of all points.
    double dErrorSquaredLimit  = mdMaxPixelErrorSquared * 4;
    double adSampsonusScores[2];
    for(int i=0; i<2; i++){
      mySE3 se3 = mvDecompositions[i].se3SecondFromFirst;
      Eigen::Matrix3d m3Essential;
      for(int j=0; j<3; j++){

        Eigen::Vector3d trans = se3.get_translation();

        Eigen::Vector3d rot_T;
        rot_T(0) = se3.get_rotation().get_matrix()(0,j);
        rot_T(1) = se3.get_rotation().get_matrix()(1,j);
        rot_T(2) = se3.get_rotation().get_matrix()(2,j);

        Eigen::Vector3d sol = trans.cross(rot_T);

        m3Essential(0,j) = sol(0);
        m3Essential(1,j) = sol(1);
        m3Essential(2,j) = sol(2);
      }

      double dSumError = 0;
      for(unsigned int m=0; m < mvMatches.size(); m++ ){
        double d = SampsonusError(mvMatches[m].v2CamPlaneSecond, m3Essential, mvMatches[m].v2CamPlaneFirst);
        if(d > dErrorSquaredLimit)
          d = dErrorSquaredLimit;
        dSumError += d;
      }

      adSampsonusScores[i] = dSumError;
    }

    if(adSampsonusScores[0] <= adSampsonusScores[1])
      mvDecompositions.erase(mvDecompositions.begin() + 1);
    else
      mvDecompositions.erase(mvDecompositions.begin());
  }

//  std::cout <<"Finish mvDecompositions " << mvDecompositions.size()<< std::endl;
}


