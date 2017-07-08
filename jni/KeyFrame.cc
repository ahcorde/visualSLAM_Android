#include "KeyFrame.h"
#include "vision/ImageHandler.h"
#include "SmallBlurryImage.h"

void KeyFrame::MakeKeyFrame_Lite(cv::Mat& im, cv::Mat& imColor) {
  // Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
  // Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
  // e.g. does not perform FAST nonmax suppression. Things like that which are needed by the 
  // mapmaker but not the tracker go in MakeKeyFrame_Rest();
  
  // First, copy out the image data to the pyramid's zero level.
    im.copyTo(aLevels[0].im);
    imColor.copyTo(this->imColor);

    cv::cvtColor(this->imColor, this->imColor, CV_RGB2BGR);

  // Then, for each level...
  for(int i=0; i<LEVELS; i++) {
    Level &lev = aLevels[i];
    if(i!=0) {  // .. make a half-size image from the previous level..
      lev.im.create(aLevels[i-1].im.rows/2, aLevels[i-1].im.cols/2, CV_8UC1);
      cv::resize(aLevels[i-1].im, lev.im, lev.im.size());
    }
        
    // .. and detect and store FAST corner points.
    // I use a different threshold on each level; this is a bit of a hack
    // whose aim is to balance the different levels' relative feature densities.
    lev.vCorners.clear();
    lev.vCandidates.clear();
    lev.vMaxCorners.clear();

    if(i == 0)
      cvCornerFast_10(lev.im, lev.vCorners, 10);
    if(i == 1)
      cvCornerFast_10(lev.im, lev.vCorners, 15);
    if(i == 2)
      cvCornerFast_10(lev.im, lev.vCorners, 15);
    if(i == 3)
      cvCornerFast_10(lev.im, lev.vCorners, 10);

    // Generate row look-up-table for the FAST corner points: this speeds up
    // finding close-by corner points later on.
    unsigned int v=0;
    lev.vCornerRowLUT.clear();
    for(int y=0; y<lev.im.rows; y++) {
        while(v < lev.vCorners.size() && y > lev.vCorners[v](1))
            v++;
        lev.vCornerRowLUT.push_back(v);
    }
  }
}

void KeyFrame::MakeKeyFrame_Rest() {
  // Fills the rest of the keyframe structure needed by the mapmaker:
  // FAST nonmax suppression, generation of the list of candidates for further map points,
  // creation of the relocaliser's SmallBlurryImage.
  double gvdCandidateMinSTScore = 70;
  
  // For each level...
  for(int l=0; l<LEVELS; l++) {
    Level &lev = aLevels[l];
    // .. find those FAST corners which are maximal..
    fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);

    int border =  10;

    // .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
    // a suitably high score as Candidates, i.e. points which the mapmaker will attempt
    // to make new map points out of.
    for(vector<Eigen::Vector2d>::iterator i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++) {
      Eigen::Vector2d vec2 = *i;

      if(! (vec2(0) >=border && vec2(1) >=border && vec2(0) < lev.im.cols - border && vec2(1) < lev.im.rows - border))
        continue;


      int a, b;
      a = vec2(0);
      b = vec2(1);
      double dSTScore = FindShiTomasiScoreAtPoint(lev.im, 3, a, b);

      if(dSTScore > gvdCandidateMinSTScore){
        Candidate c;
        c.irLevelPos = vec2;
        c.dSTScore = dSTScore;
        lev.vCandidates.push_back(c);

        int indice = vec2(1)*this->imColor.step +  vec2(0)*this->imColor.channels();
        c.rColor = this->imColor.data[indice] ;
        c.gColor = this->imColor.data[indice+1] ;
        c.bColor = this->imColor.data[indice+2] ;

      }
    }
  }
  
  // Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
  pSBI = new SmallBlurryImage(*this);  
  // Relocaliser also wants the jacobians..
  pSBI->MakeJacs();
}

// The keyframe struct is quite happy with default operator=, but Level needs its own
Level& Level::operator=(const Level &rhs) {
  // Operator= should physically copy pixels
  rhs.im.copyTo(im);

  vCorners = rhs.vCorners;
  vMaxCorners = rhs.vMaxCorners;
  vCornerRowLUT = rhs.vCornerRowLUT;
  return *this;
}

// -------------------------------------------------------------
// Some useful globals defined in LevelHelpers.h live here:
Eigen::Vector3d gavLevelColors[LEVELS];

// These globals are filled in here. A single static instance of this struct is run before main()
struct LevelHelpersFiller {// Code which should be initialised on init goes here; this runs before main()
  LevelHelpersFiller() {
    for(int i=0; i<LEVELS; i++) {
	    if(i==0)  gavLevelColors[i] << 1.0, 0.0, 0.0;
	    else if(i==1)  gavLevelColors[i] << 1.0, 1.0, 0.0;
	    else if(i==2)  gavLevelColors[i] << 0.0, 1.0, 0.0;
	    else if(i==3)  gavLevelColors[i] << 0.0, 0.0, 0.7;
	    else gavLevelColors[i] << 1.0, 1.0, 0.7; // In case I ever run with LEVELS > 4      
    }
  }
};
static LevelHelpersFiller foo;







