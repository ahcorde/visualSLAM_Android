#include "cvfast.h"

//Definitions for corner detectors.
//These are internal functions and should not be called directly.
//Input checking is only done in cvCornerFast

CvPoint* icvFast9Detect(const unsigned char* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);
int* icvFast9Score(const unsigned char* i, int stride, CvPoint* corners, int num_corners, int b);

/* This defines non-strict maxima */
#define Compare(X, Y) ((X)>=(Y))

/* This is a fast, integer only, sparse nonmaximal suppression. */
/* probably only useful for FAST corner detection */
CvPoint* icvFastNonmaxSuppression(const CvPoint* corners, const int* scores, int num_corners, int* ret_num_nonmax)
{
  int num_nonmax=0;
  int last_row;
  int* row_start;
  int i, j;
  CvPoint* ret_nonmax;
  const int sz = (int)num_corners; 

  /*Point above points (roughly) to the pixel above the one of interest, if there is a feature there.*/
  int point_above = 0;
  int point_below = 0;

  
  if(num_corners < 1)  {
    *ret_num_nonmax = 0;
    return 0;
  }

  ret_nonmax = (CvPoint*)malloc(num_corners * sizeof(CvPoint));

  /* Find where each row begins
     (the corners are output in raster scan order). A beginning of -1 signifies
     that there are no corners on that row. */
  last_row = corners[num_corners-1].y;
  row_start = (int*)malloc((last_row+1)*sizeof(int));

  for(i=0; i < last_row+1; i++)
    row_start[i] = -1;
  
  {
    int prev_row = -1;
    for(i=0; i< num_corners; i++)
      if(corners[i].y != prev_row) {
        row_start[corners[i].y] = i;
        prev_row = corners[i].y;
      }
  }
  
  for(i=0; i < sz; i++)
  {
    int score = scores[i];
    CvPoint pos = corners[i];
      
    /*Check left */
    if(i > 0)
      if(corners[i-1].x == pos.x-1 && corners[i-1].y == pos.y && Compare(scores[i-1], score))
        continue;
      
    /*Check right*/
    if(i < (sz - 1))
      if(corners[i+1].x == pos.x+1 && corners[i+1].y == pos.y && Compare(scores[i+1], score))
        continue;
      
    /*Check above (if there is a valid row above)*/
    if(pos.y != 0 && row_start[pos.y - 1] != -1) 
    {
      /*Make sure that current point_above is one
        row above.*/
      if(corners[point_above].y < pos.y - 1)
        point_above = row_start[pos.y-1];
      
      /*Make point_above point to the first of the pixels above the current point,
        if it exists.*/
      for(; corners[point_above].y < pos.y && corners[point_above].x < pos.x - 1; point_above++)
      {}
      
      
      for(j=point_above; corners[j].y < pos.y && corners[j].x <= pos.x + 1; j++)
      {
        int x = corners[j].x;
        if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && Compare(scores[j], score))
          goto cont;
      }
      
    }
      
    /*Check below (if there is anything below)*/
    if(pos.y != last_row && row_start[pos.y + 1] != -1 && point_below < sz) /*Nothing below*/
    {
      if(corners[point_below].y < pos.y + 1)
        point_below = row_start[pos.y+1];
      
      /* Make point below point to one of the pixels belowthe current point, if it
         exists.*/
      for(; point_below < sz && corners[point_below].y == pos.y+1 && corners[point_below].x < pos.x - 1; point_below++)
      {}

      for(j=point_below; j < sz && corners[j].y == pos.y+1 && corners[j].x <= pos.x + 1; j++)
      {
        int x = corners[j].x;
        if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && Compare(scores[j],score))
          goto cont;
      }
    }
    
    ret_nonmax[num_nonmax++] = corners[i];
    cont:
      ;
  }

  free(row_start);
  *ret_num_nonmax = num_nonmax;
  return ret_nonmax;
}


CVAPI(void)
cvCornerFast(cv::Mat &src, int threshold, int N, int nonmax_suppression, int* ret_number_of_corners, CvPoint** ret_corners, int ** scores)
{
  CV_FUNCNAME( "cvCornerFast" );

  int rows, cols, stride;
  uchar* data;
  
  CvPoint *corners=0, *nonmax_corners=0;
  int num_corners=0, num_nonmax=0;
  *scores=0;

  //CvMat stub;

    __CV_BEGIN__;

  //if(!input_image)
  //      CV_ERROR( CV_StsNullPtr, "" );

  if(!ret_number_of_corners)
        CV_ERROR( CV_StsNullPtr, "" );

  if( N < 9 || N > 12)
        CV_ERROR( CV_StsOutOfRange, "Corner arc length must be 9, 10, 11 or 12." );
  
  
  /* Make sure the input is unsigned char */
  /* In principle, FAST can work on any type, but trees are only included for unsigned char */
//    src = (CvMat*)input_image;
//    CV_CALL( src = cvGetMat( input_image, &stub, NULL, 0 ));

//    if( CV_MAT_TYPE(src->type) != CV_8UC1 )
//        CV_ERROR( CV_StsUnsupportedFormat, "Input must be 8uC1." );

  rows = src.rows;
  cols = src.cols;
  stride = src.step;
  data = src.data;
  
  /*Detect corners*/
  corners = icvFast9Detect(data, cols, rows, stride, threshold, &num_corners);

  /*Compute scores*/
  *scores = icvFast9Score(data, stride, corners, num_corners, threshold);

  *ret_number_of_corners = num_corners;
  *ret_corners = corners;
  
  //Do nonmax suppression if need be
  if(nonmax_suppression)
  {
    nonmax_corners = icvFastNonmaxSuppression(corners, *scores, num_corners, & num_nonmax);

    *ret_corners = nonmax_corners;
    *ret_number_of_corners = num_nonmax; 

    free(corners);
  }

    __CV_END__;
}

int icvFast9CornerScore(const unsigned char* p, const int pixel[], int bstart)
{    
    int bmin = bstart;
    int bmax = 255;
    int b = (bmax + bmin)/2;
    
    /*Compute the score using binary search*/
    for(;;) {
    int cb = *p + b;
    int c_b= *p - b;


        if( p[pixel[0]] > cb)
         if( p[pixel[1]] > cb)
          if( p[pixel[2]] > cb)
           if( p[pixel[3]] > cb)
            if( p[pixel[4]] > cb)
             if( p[pixel[5]] > cb)
              if( p[pixel[6]] > cb)
               if( p[pixel[7]] > cb)
                if( p[pixel[8]] > cb)
                 goto is_a_corner;
                else
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
               else if( p[pixel[7]] < c_b)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else if( p[pixel[14]] < c_b)
                 if( p[pixel[8]] < c_b)
                  if( p[pixel[9]] < c_b)
                   if( p[pixel[10]] < c_b)
                    if( p[pixel[11]] < c_b)
                     if( p[pixel[12]] < c_b)
                      if( p[pixel[13]] < c_b)
                       if( p[pixel[15]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else if( p[pixel[6]] < c_b)
               if( p[pixel[15]] > cb)
                if( p[pixel[13]] > cb)
                 if( p[pixel[14]] > cb)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else if( p[pixel[13]] < c_b)
                 if( p[pixel[7]] < c_b)
                  if( p[pixel[8]] < c_b)
                   if( p[pixel[9]] < c_b)
                    if( p[pixel[10]] < c_b)
                     if( p[pixel[11]] < c_b)
                      if( p[pixel[12]] < c_b)
                       if( p[pixel[14]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                if( p[pixel[7]] < c_b)
                 if( p[pixel[8]] < c_b)
                  if( p[pixel[9]] < c_b)
                   if( p[pixel[10]] < c_b)
                    if( p[pixel[11]] < c_b)
                     if( p[pixel[12]] < c_b)
                      if( p[pixel[13]] < c_b)
                       if( p[pixel[14]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else if( p[pixel[13]] < c_b)
                if( p[pixel[7]] < c_b)
                 if( p[pixel[8]] < c_b)
                  if( p[pixel[9]] < c_b)
                   if( p[pixel[10]] < c_b)
                    if( p[pixel[11]] < c_b)
                     if( p[pixel[12]] < c_b)
                      if( p[pixel[14]] < c_b)
                       if( p[pixel[15]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else if( p[pixel[5]] < c_b)
              if( p[pixel[14]] > cb)
               if( p[pixel[12]] > cb)
                if( p[pixel[13]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       if( p[pixel[11]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else if( p[pixel[12]] < c_b)
                if( p[pixel[6]] < c_b)
                 if( p[pixel[7]] < c_b)
                  if( p[pixel[8]] < c_b)
                   if( p[pixel[9]] < c_b)
                    if( p[pixel[10]] < c_b)
                     if( p[pixel[11]] < c_b)
                      if( p[pixel[13]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[14]] < c_b)
               if( p[pixel[7]] < c_b)
                if( p[pixel[8]] < c_b)
                 if( p[pixel[9]] < c_b)
                  if( p[pixel[10]] < c_b)
                   if( p[pixel[11]] < c_b)
                    if( p[pixel[12]] < c_b)
                     if( p[pixel[13]] < c_b)
                      if( p[pixel[6]] < c_b)
                       goto is_a_corner;
                      else
                       if( p[pixel[15]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               if( p[pixel[6]] < c_b)
                if( p[pixel[7]] < c_b)
                 if( p[pixel[8]] < c_b)
                  if( p[pixel[9]] < c_b)
                   if( p[pixel[10]] < c_b)
                    if( p[pixel[11]] < c_b)
                     if( p[pixel[12]] < c_b)
                      if( p[pixel[13]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       if( p[pixel[11]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[12]] < c_b)
               if( p[pixel[7]] < c_b)
                if( p[pixel[8]] < c_b)
                 if( p[pixel[9]] < c_b)
                  if( p[pixel[10]] < c_b)
                   if( p[pixel[11]] < c_b)
                    if( p[pixel[13]] < c_b)
                     if( p[pixel[14]] < c_b)
                      if( p[pixel[6]] < c_b)
                       goto is_a_corner;
                      else
                       if( p[pixel[15]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else if( p[pixel[4]] < c_b)
             if( p[pixel[13]] > cb)
              if( p[pixel[11]] > cb)
               if( p[pixel[12]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[11]] < c_b)
               if( p[pixel[5]] < c_b)
                if( p[pixel[6]] < c_b)
                 if( p[pixel[7]] < c_b)
                  if( p[pixel[8]] < c_b)
                   if( p[pixel[9]] < c_b)
                    if( p[pixel[10]] < c_b)
                     if( p[pixel[12]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else if( p[pixel[13]] < c_b)
              if( p[pixel[7]] < c_b)
               if( p[pixel[8]] < c_b)
                if( p[pixel[9]] < c_b)
                 if( p[pixel[10]] < c_b)
                  if( p[pixel[11]] < c_b)
                   if( p[pixel[12]] < c_b)
                    if( p[pixel[6]] < c_b)
                     if( p[pixel[5]] < c_b)
                      goto is_a_corner;
                     else
                      if( p[pixel[14]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                    else
                     if( p[pixel[14]] < c_b)
                      if( p[pixel[15]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              if( p[pixel[5]] < c_b)
               if( p[pixel[6]] < c_b)
                if( p[pixel[7]] < c_b)
                 if( p[pixel[8]] < c_b)
                  if( p[pixel[9]] < c_b)
                   if( p[pixel[10]] < c_b)
                    if( p[pixel[11]] < c_b)
                     if( p[pixel[12]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else if( p[pixel[11]] < c_b)
              if( p[pixel[7]] < c_b)
               if( p[pixel[8]] < c_b)
                if( p[pixel[9]] < c_b)
                 if( p[pixel[10]] < c_b)
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    if( p[pixel[6]] < c_b)
                     if( p[pixel[5]] < c_b)
                      goto is_a_corner;
                     else
                      if( p[pixel[14]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                    else
                     if( p[pixel[14]] < c_b)
                      if( p[pixel[15]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
           else if( p[pixel[3]] < c_b)
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else if( p[pixel[10]] < c_b)
             if( p[pixel[7]] < c_b)
              if( p[pixel[8]] < c_b)
               if( p[pixel[9]] < c_b)
                if( p[pixel[11]] < c_b)
                 if( p[pixel[6]] < c_b)
                  if( p[pixel[5]] < c_b)
                   if( p[pixel[4]] < c_b)
                    goto is_a_corner;
                   else
                    if( p[pixel[12]] < c_b)
                     if( p[pixel[13]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[12]] < c_b)
                    if( p[pixel[13]] < c_b)
                     if( p[pixel[14]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    if( p[pixel[14]] < c_b)
                     if( p[pixel[15]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else if( p[pixel[10]] < c_b)
             if( p[pixel[7]] < c_b)
              if( p[pixel[8]] < c_b)
               if( p[pixel[9]] < c_b)
                if( p[pixel[11]] < c_b)
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[5]] < c_b)
                    if( p[pixel[4]] < c_b)
                     goto is_a_corner;
                    else
                     if( p[pixel[13]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    if( p[pixel[13]] < c_b)
                     if( p[pixel[14]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[13]] < c_b)
                    if( p[pixel[14]] < c_b)
                     if( p[pixel[15]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
          else if( p[pixel[2]] < c_b)
           if( p[pixel[9]] > cb)
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else if( p[pixel[9]] < c_b)
            if( p[pixel[7]] < c_b)
             if( p[pixel[8]] < c_b)
              if( p[pixel[10]] < c_b)
               if( p[pixel[6]] < c_b)
                if( p[pixel[5]] < c_b)
                 if( p[pixel[4]] < c_b)
                  if( p[pixel[3]] < c_b)
                   goto is_a_corner;
                  else
                   if( p[pixel[11]] < c_b)
                    if( p[pixel[12]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[11]] < c_b)
                   if( p[pixel[12]] < c_b)
                    if( p[pixel[13]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[11]] < c_b)
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    if( p[pixel[14]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[11]] < c_b)
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[13]] < c_b)
                   if( p[pixel[14]] < c_b)
                    if( p[pixel[15]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           if( p[pixel[9]] > cb)
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else if( p[pixel[9]] < c_b)
            if( p[pixel[7]] < c_b)
             if( p[pixel[8]] < c_b)
              if( p[pixel[10]] < c_b)
               if( p[pixel[11]] < c_b)
                if( p[pixel[6]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[4]] < c_b)
                   if( p[pixel[3]] < c_b)
                    goto is_a_corner;
                   else
                    if( p[pixel[12]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[12]] < c_b)
                    if( p[pixel[13]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    if( p[pixel[14]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[13]] < c_b)
                   if( p[pixel[14]] < c_b)
                    if( p[pixel[15]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
         else if( p[pixel[1]] < c_b)
          if( p[pixel[8]] > cb)
           if( p[pixel[9]] > cb)
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[2]] > cb)
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
           if( p[pixel[7]] < c_b)
            if( p[pixel[9]] < c_b)
             if( p[pixel[6]] < c_b)
              if( p[pixel[5]] < c_b)
               if( p[pixel[4]] < c_b)
                if( p[pixel[3]] < c_b)
                 if( p[pixel[2]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[10]] < c_b)
                   if( p[pixel[11]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[10]] < c_b)
                  if( p[pixel[11]] < c_b)
                   if( p[pixel[12]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[10]] < c_b)
                 if( p[pixel[11]] < c_b)
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[10]] < c_b)
                if( p[pixel[11]] < c_b)
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[13]] < c_b)
                   if( p[pixel[14]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[10]] < c_b)
               if( p[pixel[11]] < c_b)
                if( p[pixel[12]] < c_b)
                 if( p[pixel[13]] < c_b)
                  if( p[pixel[14]] < c_b)
                   if( p[pixel[15]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
         else
          if( p[pixel[8]] > cb)
           if( p[pixel[9]] > cb)
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[2]] > cb)
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
           if( p[pixel[7]] < c_b)
            if( p[pixel[9]] < c_b)
             if( p[pixel[10]] < c_b)
              if( p[pixel[6]] < c_b)
               if( p[pixel[5]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[3]] < c_b)
                  if( p[pixel[2]] < c_b)
                   goto is_a_corner;
                  else
                   if( p[pixel[11]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[11]] < c_b)
                   if( p[pixel[12]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[11]] < c_b)
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[11]] < c_b)
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[13]] < c_b)
                   if( p[pixel[14]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[11]] < c_b)
                if( p[pixel[12]] < c_b)
                 if( p[pixel[13]] < c_b)
                  if( p[pixel[14]] < c_b)
                   if( p[pixel[15]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
        else if( p[pixel[0]] < c_b)
         if( p[pixel[1]] > cb)
          if( p[pixel[8]] > cb)
           if( p[pixel[7]] > cb)
            if( p[pixel[9]] > cb)
             if( p[pixel[6]] > cb)
              if( p[pixel[5]] > cb)
               if( p[pixel[4]] > cb)
                if( p[pixel[3]] > cb)
                 if( p[pixel[2]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[10]] > cb)
                   if( p[pixel[11]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[10]] > cb)
                  if( p[pixel[11]] > cb)
                   if( p[pixel[12]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[10]] > cb)
                 if( p[pixel[11]] > cb)
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[10]] > cb)
                if( p[pixel[11]] > cb)
                 if( p[pixel[12]] > cb)
                  if( p[pixel[13]] > cb)
                   if( p[pixel[14]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[10]] > cb)
               if( p[pixel[11]] > cb)
                if( p[pixel[12]] > cb)
                 if( p[pixel[13]] > cb)
                  if( p[pixel[14]] > cb)
                   if( p[pixel[15]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
           if( p[pixel[9]] < c_b)
            if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[2]] < c_b)
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
         else if( p[pixel[1]] < c_b)
          if( p[pixel[2]] > cb)
           if( p[pixel[9]] > cb)
            if( p[pixel[7]] > cb)
             if( p[pixel[8]] > cb)
              if( p[pixel[10]] > cb)
               if( p[pixel[6]] > cb)
                if( p[pixel[5]] > cb)
                 if( p[pixel[4]] > cb)
                  if( p[pixel[3]] > cb)
                   goto is_a_corner;
                  else
                   if( p[pixel[11]] > cb)
                    if( p[pixel[12]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[11]] > cb)
                   if( p[pixel[12]] > cb)
                    if( p[pixel[13]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[11]] > cb)
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    if( p[pixel[14]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[11]] > cb)
                 if( p[pixel[12]] > cb)
                  if( p[pixel[13]] > cb)
                   if( p[pixel[14]] > cb)
                    if( p[pixel[15]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else if( p[pixel[9]] < c_b)
            if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else if( p[pixel[2]] < c_b)
           if( p[pixel[3]] > cb)
            if( p[pixel[10]] > cb)
             if( p[pixel[7]] > cb)
              if( p[pixel[8]] > cb)
               if( p[pixel[9]] > cb)
                if( p[pixel[11]] > cb)
                 if( p[pixel[6]] > cb)
                  if( p[pixel[5]] > cb)
                   if( p[pixel[4]] > cb)
                    goto is_a_corner;
                   else
                    if( p[pixel[12]] > cb)
                     if( p[pixel[13]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[12]] > cb)
                    if( p[pixel[13]] > cb)
                     if( p[pixel[14]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    if( p[pixel[14]] > cb)
                     if( p[pixel[15]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else if( p[pixel[3]] < c_b)
            if( p[pixel[4]] > cb)
             if( p[pixel[13]] > cb)
              if( p[pixel[7]] > cb)
               if( p[pixel[8]] > cb)
                if( p[pixel[9]] > cb)
                 if( p[pixel[10]] > cb)
                  if( p[pixel[11]] > cb)
                   if( p[pixel[12]] > cb)
                    if( p[pixel[6]] > cb)
                     if( p[pixel[5]] > cb)
                      goto is_a_corner;
                     else
                      if( p[pixel[14]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                    else
                     if( p[pixel[14]] > cb)
                      if( p[pixel[15]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else if( p[pixel[13]] < c_b)
              if( p[pixel[11]] > cb)
               if( p[pixel[5]] > cb)
                if( p[pixel[6]] > cb)
                 if( p[pixel[7]] > cb)
                  if( p[pixel[8]] > cb)
                   if( p[pixel[9]] > cb)
                    if( p[pixel[10]] > cb)
                     if( p[pixel[12]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[11]] < c_b)
               if( p[pixel[12]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              if( p[pixel[5]] > cb)
               if( p[pixel[6]] > cb)
                if( p[pixel[7]] > cb)
                 if( p[pixel[8]] > cb)
                  if( p[pixel[9]] > cb)
                   if( p[pixel[10]] > cb)
                    if( p[pixel[11]] > cb)
                     if( p[pixel[12]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else if( p[pixel[4]] < c_b)
             if( p[pixel[5]] > cb)
              if( p[pixel[14]] > cb)
               if( p[pixel[7]] > cb)
                if( p[pixel[8]] > cb)
                 if( p[pixel[9]] > cb)
                  if( p[pixel[10]] > cb)
                   if( p[pixel[11]] > cb)
                    if( p[pixel[12]] > cb)
                     if( p[pixel[13]] > cb)
                      if( p[pixel[6]] > cb)
                       goto is_a_corner;
                      else
                       if( p[pixel[15]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[14]] < c_b)
               if( p[pixel[12]] > cb)
                if( p[pixel[6]] > cb)
                 if( p[pixel[7]] > cb)
                  if( p[pixel[8]] > cb)
                   if( p[pixel[9]] > cb)
                    if( p[pixel[10]] > cb)
                     if( p[pixel[11]] > cb)
                      if( p[pixel[13]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else if( p[pixel[12]] < c_b)
                if( p[pixel[13]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       if( p[pixel[11]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               if( p[pixel[6]] > cb)
                if( p[pixel[7]] > cb)
                 if( p[pixel[8]] > cb)
                  if( p[pixel[9]] > cb)
                   if( p[pixel[10]] > cb)
                    if( p[pixel[11]] > cb)
                     if( p[pixel[12]] > cb)
                      if( p[pixel[13]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else if( p[pixel[5]] < c_b)
              if( p[pixel[6]] > cb)
               if( p[pixel[15]] < c_b)
                if( p[pixel[13]] > cb)
                 if( p[pixel[7]] > cb)
                  if( p[pixel[8]] > cb)
                   if( p[pixel[9]] > cb)
                    if( p[pixel[10]] > cb)
                     if( p[pixel[11]] > cb)
                      if( p[pixel[12]] > cb)
                       if( p[pixel[14]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else if( p[pixel[13]] < c_b)
                 if( p[pixel[14]] < c_b)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                if( p[pixel[7]] > cb)
                 if( p[pixel[8]] > cb)
                  if( p[pixel[9]] > cb)
                   if( p[pixel[10]] > cb)
                    if( p[pixel[11]] > cb)
                     if( p[pixel[12]] > cb)
                      if( p[pixel[13]] > cb)
                       if( p[pixel[14]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else if( p[pixel[6]] < c_b)
               if( p[pixel[7]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[8]] > cb)
                  if( p[pixel[9]] > cb)
                   if( p[pixel[10]] > cb)
                    if( p[pixel[11]] > cb)
                     if( p[pixel[12]] > cb)
                      if( p[pixel[13]] > cb)
                       if( p[pixel[15]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else if( p[pixel[7]] < c_b)
                if( p[pixel[8]] < c_b)
                 goto is_a_corner;
                else
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[13]] > cb)
                if( p[pixel[7]] > cb)
                 if( p[pixel[8]] > cb)
                  if( p[pixel[9]] > cb)
                   if( p[pixel[10]] > cb)
                    if( p[pixel[11]] > cb)
                     if( p[pixel[12]] > cb)
                      if( p[pixel[14]] > cb)
                       if( p[pixel[15]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[12]] > cb)
               if( p[pixel[7]] > cb)
                if( p[pixel[8]] > cb)
                 if( p[pixel[9]] > cb)
                  if( p[pixel[10]] > cb)
                   if( p[pixel[11]] > cb)
                    if( p[pixel[13]] > cb)
                     if( p[pixel[14]] > cb)
                      if( p[pixel[6]] > cb)
                       goto is_a_corner;
                      else
                       if( p[pixel[15]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       if( p[pixel[11]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             if( p[pixel[11]] > cb)
              if( p[pixel[7]] > cb)
               if( p[pixel[8]] > cb)
                if( p[pixel[9]] > cb)
                 if( p[pixel[10]] > cb)
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    if( p[pixel[6]] > cb)
                     if( p[pixel[5]] > cb)
                      goto is_a_corner;
                     else
                      if( p[pixel[14]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                    else
                     if( p[pixel[14]] > cb)
                      if( p[pixel[15]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
           else
            if( p[pixel[10]] > cb)
             if( p[pixel[7]] > cb)
              if( p[pixel[8]] > cb)
               if( p[pixel[9]] > cb)
                if( p[pixel[11]] > cb)
                 if( p[pixel[12]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[5]] > cb)
                    if( p[pixel[4]] > cb)
                     goto is_a_corner;
                    else
                     if( p[pixel[13]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    if( p[pixel[13]] > cb)
                     if( p[pixel[14]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[13]] > cb)
                    if( p[pixel[14]] > cb)
                     if( p[pixel[15]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
          else
           if( p[pixel[9]] > cb)
            if( p[pixel[7]] > cb)
             if( p[pixel[8]] > cb)
              if( p[pixel[10]] > cb)
               if( p[pixel[11]] > cb)
                if( p[pixel[6]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[4]] > cb)
                   if( p[pixel[3]] > cb)
                    goto is_a_corner;
                   else
                    if( p[pixel[12]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[12]] > cb)
                    if( p[pixel[13]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    if( p[pixel[14]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[12]] > cb)
                  if( p[pixel[13]] > cb)
                   if( p[pixel[14]] > cb)
                    if( p[pixel[15]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else if( p[pixel[9]] < c_b)
            if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
         else
          if( p[pixel[8]] > cb)
           if( p[pixel[7]] > cb)
            if( p[pixel[9]] > cb)
             if( p[pixel[10]] > cb)
              if( p[pixel[6]] > cb)
               if( p[pixel[5]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[3]] > cb)
                  if( p[pixel[2]] > cb)
                   goto is_a_corner;
                  else
                   if( p[pixel[11]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[11]] > cb)
                   if( p[pixel[12]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[11]] > cb)
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[11]] > cb)
                 if( p[pixel[12]] > cb)
                  if( p[pixel[13]] > cb)
                   if( p[pixel[14]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[11]] > cb)
                if( p[pixel[12]] > cb)
                 if( p[pixel[13]] > cb)
                  if( p[pixel[14]] > cb)
                   if( p[pixel[15]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
           if( p[pixel[9]] < c_b)
            if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[2]] < c_b)
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
        else
         if( p[pixel[7]] > cb)
          if( p[pixel[8]] > cb)
           if( p[pixel[9]] > cb)
            if( p[pixel[6]] > cb)
             if( p[pixel[5]] > cb)
              if( p[pixel[4]] > cb)
               if( p[pixel[3]] > cb)
                if( p[pixel[2]] > cb)
                 if( p[pixel[1]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[10]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[10]] > cb)
                  if( p[pixel[11]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[10]] > cb)
                 if( p[pixel[11]] > cb)
                  if( p[pixel[12]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[10]] > cb)
                if( p[pixel[11]] > cb)
                 if( p[pixel[12]] > cb)
                  if( p[pixel[13]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[10]] > cb)
               if( p[pixel[11]] > cb)
                if( p[pixel[12]] > cb)
                 if( p[pixel[13]] > cb)
                  if( p[pixel[14]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             if( p[pixel[10]] > cb)
              if( p[pixel[11]] > cb)
               if( p[pixel[12]] > cb)
                if( p[pixel[13]] > cb)
                 if( p[pixel[14]] > cb)
                  if( p[pixel[15]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
         else if( p[pixel[7]] < c_b)
          if( p[pixel[8]] < c_b)
           if( p[pixel[9]] < c_b)
            if( p[pixel[6]] < c_b)
             if( p[pixel[5]] < c_b)
              if( p[pixel[4]] < c_b)
               if( p[pixel[3]] < c_b)
                if( p[pixel[2]] < c_b)
                 if( p[pixel[1]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[10]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[10]] < c_b)
                  if( p[pixel[11]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[10]] < c_b)
                 if( p[pixel[11]] < c_b)
                  if( p[pixel[12]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[10]] < c_b)
                if( p[pixel[11]] < c_b)
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[13]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[10]] < c_b)
               if( p[pixel[11]] < c_b)
                if( p[pixel[12]] < c_b)
                 if( p[pixel[13]] < c_b)
                  if( p[pixel[14]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             if( p[pixel[10]] < c_b)
              if( p[pixel[11]] < c_b)
               if( p[pixel[12]] < c_b)
                if( p[pixel[13]] < c_b)
                 if( p[pixel[14]] < c_b)
                  if( p[pixel[15]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
         else
          goto is_not_a_corner;

    is_a_corner:
      bmin=b;
      goto end_if;

    is_not_a_corner:
      bmax=b;
      goto end_if;

    end_if:

    if(bmin == bmax - 1 || bmin == bmax)
      return bmin;
    b = (bmin + bmax) / 2;
    }
}

static void icvMakeOffsets(int pixel[], int row_stride)
{
        pixel[0] = 0 + row_stride * 3;
        pixel[1] = 1 + row_stride * 3;
        pixel[2] = 2 + row_stride * 2;
        pixel[3] = 3 + row_stride * 1;
        pixel[4] = 3 + row_stride * 0;
        pixel[5] = 3 + row_stride * -1;
        pixel[6] = 2 + row_stride * -2;
        pixel[7] = 1 + row_stride * -3;
        pixel[8] = 0 + row_stride * -3;
        pixel[9] = -1 + row_stride * -3;
        pixel[10] = -2 + row_stride * -2;
        pixel[11] = -3 + row_stride * -1;
        pixel[12] = -3 + row_stride * 0;
        pixel[13] = -3 + row_stride * 1;
        pixel[14] = -2 + row_stride * 2;
        pixel[15] = -1 + row_stride * 3;
}



int* icvFast9Score(const unsigned char* i, int stride, CvPoint* corners, int num_corners, int b)
{  
  int* scores = (int*)malloc(sizeof(int)* num_corners);
  int n;

  int pixel[16];
  icvMakeOffsets(pixel, stride);

    for(n=0; n < num_corners; n++)
        scores[n] = icvFast9CornerScore(i + corners[n].y*stride + corners[n].x, pixel, b);

  return scores;
}


CvPoint* icvFast9Detect(const unsigned char* im, int xsize, int ysize, int stride, int b, int* ret_num_corners)
{
  int num_corners=0;
  CvPoint* ret_corners;
  int rsize=512;
  int pixel[16];
  int x, y;

  ret_corners = (CvPoint*)malloc(sizeof(CvPoint)*rsize);
  icvMakeOffsets(pixel, stride);

  for(y=3; y < ysize - 3; y++)
    for(x=3; x < xsize - 3; x++)
    {
      const unsigned char* p = im + y*stride + x;
    
      int cb = *p + b;
      int c_b= *p - b;
        if(p[pixel[0]] > cb)
         if(p[pixel[1]] > cb)
          if(p[pixel[2]] > cb)
           if(p[pixel[3]] > cb)
            if(p[pixel[4]] > cb)
             if(p[pixel[5]] > cb)
              if(p[pixel[6]] > cb)
               if(p[pixel[7]] > cb)
                if(p[pixel[8]] > cb)
                 {}
                else
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
               else if(p[pixel[7]] < c_b)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
                else if(p[pixel[14]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[13]] < c_b)
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
                else
                 continue;
              else if(p[pixel[6]] < c_b)
               if(p[pixel[15]] > cb)
                if(p[pixel[13]] > cb)
                 if(p[pixel[14]] > cb)
                  {}
                 else
                  continue;
                else if(p[pixel[13]] < c_b)
                 if(p[pixel[7]] < c_b)
                  if(p[pixel[8]] < c_b)
                   if(p[pixel[9]] < c_b)
                    if(p[pixel[10]] < c_b)
                     if(p[pixel[11]] < c_b)
                      if(p[pixel[12]] < c_b)
                       if(p[pixel[14]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[13]] < c_b)
                       if(p[pixel[14]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[13]] < c_b)
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[14]] < c_b)
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else if(p[pixel[5]] < c_b)
              if(p[pixel[14]] > cb)
               if(p[pixel[12]] > cb)
                if(p[pixel[13]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       if(p[pixel[11]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else if(p[pixel[12]] < c_b)
                if(p[pixel[6]] < c_b)
                 if(p[pixel[7]] < c_b)
                  if(p[pixel[8]] < c_b)
                   if(p[pixel[9]] < c_b)
                    if(p[pixel[10]] < c_b)
                     if(p[pixel[11]] < c_b)
                      if(p[pixel[13]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[14]] < c_b)
               if(p[pixel[7]] < c_b)
                if(p[pixel[8]] < c_b)
                 if(p[pixel[9]] < c_b)
                  if(p[pixel[10]] < c_b)
                   if(p[pixel[11]] < c_b)
                    if(p[pixel[12]] < c_b)
                     if(p[pixel[13]] < c_b)
                      if(p[pixel[6]] < c_b)
                       {}
                      else
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               if(p[pixel[6]] < c_b)
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[13]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       if(p[pixel[11]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[12]] < c_b)
               if(p[pixel[7]] < c_b)
                if(p[pixel[8]] < c_b)
                 if(p[pixel[9]] < c_b)
                  if(p[pixel[10]] < c_b)
                   if(p[pixel[11]] < c_b)
                    if(p[pixel[13]] < c_b)
                     if(p[pixel[14]] < c_b)
                      if(p[pixel[6]] < c_b)
                       {}
                      else
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else if(p[pixel[4]] < c_b)
             if(p[pixel[13]] > cb)
              if(p[pixel[11]] > cb)
               if(p[pixel[12]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else if(p[pixel[11]] < c_b)
               if(p[pixel[5]] < c_b)
                if(p[pixel[6]] < c_b)
                 if(p[pixel[7]] < c_b)
                  if(p[pixel[8]] < c_b)
                   if(p[pixel[9]] < c_b)
                    if(p[pixel[10]] < c_b)
                     if(p[pixel[12]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[13]] < c_b)
              if(p[pixel[7]] < c_b)
               if(p[pixel[8]] < c_b)
                if(p[pixel[9]] < c_b)
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[6]] < c_b)
                     if(p[pixel[5]] < c_b)
                      {}
                     else
                      if(p[pixel[14]] < c_b)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] < c_b)
                      if(p[pixel[15]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              if(p[pixel[5]] < c_b)
               if(p[pixel[6]] < c_b)
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[11]] < c_b)
              if(p[pixel[7]] < c_b)
               if(p[pixel[8]] < c_b)
                if(p[pixel[9]] < c_b)
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[6]] < c_b)
                     if(p[pixel[5]] < c_b)
                      {}
                     else
                      if(p[pixel[14]] < c_b)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] < c_b)
                      if(p[pixel[15]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else if(p[pixel[3]] < c_b)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[7]] < c_b)
              if(p[pixel[8]] < c_b)
               if(p[pixel[9]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[6]] < c_b)
                  if(p[pixel[5]] < c_b)
                   if(p[pixel[4]] < c_b)
                    {}
                   else
                    if(p[pixel[12]] < c_b)
                     if(p[pixel[13]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[13]] < c_b)
                     if(p[pixel[14]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     if(p[pixel[15]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[7]] < c_b)
              if(p[pixel[8]] < c_b)
               if(p[pixel[9]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[5]] < c_b)
                    if(p[pixel[4]] < c_b)
                     {}
                    else
                     if(p[pixel[13]] < c_b)
                      {}
                     else
                      continue;
                   else
                    if(p[pixel[13]] < c_b)
                     if(p[pixel[14]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     if(p[pixel[15]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
          else if(p[pixel[2]] < c_b)
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[7]] < c_b)
             if(p[pixel[8]] < c_b)
              if(p[pixel[10]] < c_b)
               if(p[pixel[6]] < c_b)
                if(p[pixel[5]] < c_b)
                 if(p[pixel[4]] < c_b)
                  if(p[pixel[3]] < c_b)
                   {}
                  else
                   if(p[pixel[11]] < c_b)
                    if(p[pixel[12]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[13]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    if(p[pixel[15]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[7]] < c_b)
             if(p[pixel[8]] < c_b)
              if(p[pixel[10]] < c_b)
               if(p[pixel[11]] < c_b)
                if(p[pixel[6]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[4]] < c_b)
                   if(p[pixel[3]] < c_b)
                    {}
                   else
                    if(p[pixel[12]] < c_b)
                     {}
                    else
                     continue;
                  else
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[13]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    if(p[pixel[15]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else if(p[pixel[1]] < c_b)
          if(p[pixel[8]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] > cb)
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[7]] < c_b)
            if(p[pixel[9]] < c_b)
             if(p[pixel[6]] < c_b)
              if(p[pixel[5]] < c_b)
               if(p[pixel[4]] < c_b)
                if(p[pixel[3]] < c_b)
                 if(p[pixel[2]] < c_b)
                  {}
                 else
                  if(p[pixel[10]] < c_b)
                   if(p[pixel[11]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] < c_b)
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] < c_b)
               if(p[pixel[11]] < c_b)
                if(p[pixel[12]] < c_b)
                 if(p[pixel[13]] < c_b)
                  if(p[pixel[14]] < c_b)
                   if(p[pixel[15]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          if(p[pixel[8]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] > cb)
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[7]] < c_b)
            if(p[pixel[9]] < c_b)
             if(p[pixel[10]] < c_b)
              if(p[pixel[6]] < c_b)
               if(p[pixel[5]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[3]] < c_b)
                  if(p[pixel[2]] < c_b)
                   {}
                  else
                   if(p[pixel[11]] < c_b)
                    {}
                   else
                    continue;
                 else
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[11]] < c_b)
                if(p[pixel[12]] < c_b)
                 if(p[pixel[13]] < c_b)
                  if(p[pixel[14]] < c_b)
                   if(p[pixel[15]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
        else if(p[pixel[0]] < c_b)
         if(p[pixel[1]] > cb)
          if(p[pixel[8]] > cb)
           if(p[pixel[7]] > cb)
            if(p[pixel[9]] > cb)
             if(p[pixel[6]] > cb)
              if(p[pixel[5]] > cb)
               if(p[pixel[4]] > cb)
                if(p[pixel[3]] > cb)
                 if(p[pixel[2]] > cb)
                  {}
                 else
                  if(p[pixel[10]] > cb)
                   if(p[pixel[11]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[10]] > cb)
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] > cb)
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] > cb)
               if(p[pixel[11]] > cb)
                if(p[pixel[12]] > cb)
                 if(p[pixel[13]] > cb)
                  if(p[pixel[14]] > cb)
                   if(p[pixel[15]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] < c_b)
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else if(p[pixel[1]] < c_b)
          if(p[pixel[2]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[7]] > cb)
             if(p[pixel[8]] > cb)
              if(p[pixel[10]] > cb)
               if(p[pixel[6]] > cb)
                if(p[pixel[5]] > cb)
                 if(p[pixel[4]] > cb)
                  if(p[pixel[3]] > cb)
                   {}
                  else
                   if(p[pixel[11]] > cb)
                    if(p[pixel[12]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    if(p[pixel[13]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    if(p[pixel[15]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[2]] < c_b)
           if(p[pixel[3]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[7]] > cb)
              if(p[pixel[8]] > cb)
               if(p[pixel[9]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[6]] > cb)
                  if(p[pixel[5]] > cb)
                   if(p[pixel[4]] > cb)
                    {}
                   else
                    if(p[pixel[12]] > cb)
                     if(p[pixel[13]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[12]] > cb)
                    if(p[pixel[13]] > cb)
                     if(p[pixel[14]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     if(p[pixel[15]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[3]] < c_b)
            if(p[pixel[4]] > cb)
             if(p[pixel[13]] > cb)
              if(p[pixel[7]] > cb)
               if(p[pixel[8]] > cb)
                if(p[pixel[9]] > cb)
                 if(p[pixel[10]] > cb)
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    if(p[pixel[6]] > cb)
                     if(p[pixel[5]] > cb)
                      {}
                     else
                      if(p[pixel[14]] > cb)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] > cb)
                      if(p[pixel[15]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[13]] < c_b)
              if(p[pixel[11]] > cb)
               if(p[pixel[5]] > cb)
                if(p[pixel[6]] > cb)
                 if(p[pixel[7]] > cb)
                  if(p[pixel[8]] > cb)
                   if(p[pixel[9]] > cb)
                    if(p[pixel[10]] > cb)
                     if(p[pixel[12]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[11]] < c_b)
               if(p[pixel[12]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              if(p[pixel[5]] > cb)
               if(p[pixel[6]] > cb)
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else if(p[pixel[4]] < c_b)
             if(p[pixel[5]] > cb)
              if(p[pixel[14]] > cb)
               if(p[pixel[7]] > cb)
                if(p[pixel[8]] > cb)
                 if(p[pixel[9]] > cb)
                  if(p[pixel[10]] > cb)
                   if(p[pixel[11]] > cb)
                    if(p[pixel[12]] > cb)
                     if(p[pixel[13]] > cb)
                      if(p[pixel[6]] > cb)
                       {}
                      else
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[14]] < c_b)
               if(p[pixel[12]] > cb)
                if(p[pixel[6]] > cb)
                 if(p[pixel[7]] > cb)
                  if(p[pixel[8]] > cb)
                   if(p[pixel[9]] > cb)
                    if(p[pixel[10]] > cb)
                     if(p[pixel[11]] > cb)
                      if(p[pixel[13]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[12]] < c_b)
                if(p[pixel[13]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       if(p[pixel[11]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               if(p[pixel[6]] > cb)
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[13]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else if(p[pixel[5]] < c_b)
              if(p[pixel[6]] > cb)
               if(p[pixel[15]] < c_b)
                if(p[pixel[13]] > cb)
                 if(p[pixel[7]] > cb)
                  if(p[pixel[8]] > cb)
                   if(p[pixel[9]] > cb)
                    if(p[pixel[10]] > cb)
                     if(p[pixel[11]] > cb)
                      if(p[pixel[12]] > cb)
                       if(p[pixel[14]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(p[pixel[13]] < c_b)
                 if(p[pixel[14]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
               else
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[13]] > cb)
                       if(p[pixel[14]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else if(p[pixel[6]] < c_b)
               if(p[pixel[7]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[13]] > cb)
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[7]] < c_b)
                if(p[pixel[8]] < c_b)
                 {}
                else
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
               else
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[13]] > cb)
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[14]] > cb)
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[12]] > cb)
               if(p[pixel[7]] > cb)
                if(p[pixel[8]] > cb)
                 if(p[pixel[9]] > cb)
                  if(p[pixel[10]] > cb)
                   if(p[pixel[11]] > cb)
                    if(p[pixel[13]] > cb)
                     if(p[pixel[14]] > cb)
                      if(p[pixel[6]] > cb)
                       {}
                      else
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       if(p[pixel[11]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[11]] > cb)
              if(p[pixel[7]] > cb)
               if(p[pixel[8]] > cb)
                if(p[pixel[9]] > cb)
                 if(p[pixel[10]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[6]] > cb)
                     if(p[pixel[5]] > cb)
                      {}
                     else
                      if(p[pixel[14]] > cb)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] > cb)
                      if(p[pixel[15]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            if(p[pixel[10]] > cb)
             if(p[pixel[7]] > cb)
              if(p[pixel[8]] > cb)
               if(p[pixel[9]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[5]] > cb)
                    if(p[pixel[4]] > cb)
                     {}
                    else
                     if(p[pixel[13]] > cb)
                      {}
                     else
                      continue;
                   else
                    if(p[pixel[13]] > cb)
                     if(p[pixel[14]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     if(p[pixel[15]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
          else
           if(p[pixel[9]] > cb)
            if(p[pixel[7]] > cb)
             if(p[pixel[8]] > cb)
              if(p[pixel[10]] > cb)
               if(p[pixel[11]] > cb)
                if(p[pixel[6]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[4]] > cb)
                   if(p[pixel[3]] > cb)
                    {}
                   else
                    if(p[pixel[12]] > cb)
                     {}
                    else
                     continue;
                  else
                   if(p[pixel[12]] > cb)
                    if(p[pixel[13]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    if(p[pixel[15]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else
          if(p[pixel[8]] > cb)
           if(p[pixel[7]] > cb)
            if(p[pixel[9]] > cb)
             if(p[pixel[10]] > cb)
              if(p[pixel[6]] > cb)
               if(p[pixel[5]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[3]] > cb)
                  if(p[pixel[2]] > cb)
                   {}
                  else
                   if(p[pixel[11]] > cb)
                    {}
                   else
                    continue;
                 else
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[11]] > cb)
                if(p[pixel[12]] > cb)
                 if(p[pixel[13]] > cb)
                  if(p[pixel[14]] > cb)
                   if(p[pixel[15]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] < c_b)
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
        else
         if(p[pixel[7]] > cb)
          if(p[pixel[8]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[6]] > cb)
             if(p[pixel[5]] > cb)
              if(p[pixel[4]] > cb)
               if(p[pixel[3]] > cb)
                if(p[pixel[2]] > cb)
                 if(p[pixel[1]] > cb)
                  {}
                 else
                  if(p[pixel[10]] > cb)
                   {}
                  else
                   continue;
                else
                 if(p[pixel[10]] > cb)
                  if(p[pixel[11]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] > cb)
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] > cb)
               if(p[pixel[11]] > cb)
                if(p[pixel[12]] > cb)
                 if(p[pixel[13]] > cb)
                  if(p[pixel[14]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[10]] > cb)
              if(p[pixel[11]] > cb)
               if(p[pixel[12]] > cb)
                if(p[pixel[13]] > cb)
                 if(p[pixel[14]] > cb)
                  if(p[pixel[15]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            continue;
          else
           continue;
         else if(p[pixel[7]] < c_b)
          if(p[pixel[8]] < c_b)
           if(p[pixel[9]] < c_b)
            if(p[pixel[6]] < c_b)
             if(p[pixel[5]] < c_b)
              if(p[pixel[4]] < c_b)
               if(p[pixel[3]] < c_b)
                if(p[pixel[2]] < c_b)
                 if(p[pixel[1]] < c_b)
                  {}
                 else
                  if(p[pixel[10]] < c_b)
                   {}
                  else
                   continue;
                else
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[11]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] < c_b)
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] < c_b)
               if(p[pixel[11]] < c_b)
                if(p[pixel[12]] < c_b)
                 if(p[pixel[13]] < c_b)
                  if(p[pixel[14]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[10]] < c_b)
              if(p[pixel[11]] < c_b)
               if(p[pixel[12]] < c_b)
                if(p[pixel[13]] < c_b)
                 if(p[pixel[14]] < c_b)
                  if(p[pixel[15]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            continue;
          else
           continue;
         else
          continue;
      if(num_corners == rsize)
      {
        rsize*=2;
        ret_corners = (CvPoint*)realloc(ret_corners, sizeof(CvPoint)*rsize);
      }
      ret_corners[num_corners].x = x;
      ret_corners[num_corners].y = y;
      num_corners++;
    }
  
  *ret_num_corners = num_corners;
  return ret_corners;

}


/*TODOTODO*/

CVAPI(void) cvCornerFast_10(cv::Mat &src, std::vector<Eigen::Vector2d> &corners, int threshold)
{
  int y, cb, c_b;
  const unsigned char *line_max, *line_min;
  const unsigned char *cache_0;

  int pixel[16] = {
    0 + src.step * 3,
    1 + src.step * 3,
    2 + src.step * 2,
    3 + src.step * 1,
    3 + src.step * 0,
    3 + src.step * -1,
    2 + src.step * -2,
    1 + src.step * -3,
    0 + src.step * -3,
    -1 + src.step * -3,
    -2 + src.step * -2,
    -3 + src.step * -1,
    -3 + src.step * 0,
    -3 + src.step * 1,
    -2 + src.step * 2,
    -1 + src.step * 3,
  };

  for(y = 3 ; y < src.rows - 3; y++)
  {
    cache_0 = src.data + y*src.step + 3;
    line_min = cache_0 - 3;
    line_max = src.data + y*src.step + (src.cols - 3);

    for(; cache_0 < line_max;cache_0++)
    {
      cb = *cache_0 + threshold;
      c_b= *cache_0 - threshold;
  
      if(*(cache_0 + pixel[0]) > cb)
       if(*(cache_0 + pixel[8]) > cb)
        if(*(cache_0 + pixel[3]) > cb)
         if(*(cache_0 + pixel[5]) > cb)
          if(*(cache_0 + pixel[2]) > cb)
           if(*(cache_0 + pixel[6]) > cb)
            if(*(cache_0 + 3) > cb)
             if(*(cache_0 + pixel[7]) > cb)
              if(*(cache_0 + pixel[1]) > cb)
               if(*(cache_0 + pixel[9]) > cb)
                goto success;
               else
                if(*(cache_0 + pixel[15]) > cb)
                 goto success;
                else
                 continue;
              else if(*(cache_0 + pixel[1]) < c_b)
               if(*(cache_0 + pixel[9]) > cb)
                if(*(cache_0 + pixel[10]) > cb)
                 if(*(cache_0 + pixel[11]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               if(*(cache_0 + pixel[11]) > cb)
                if(*(cache_0 + pixel[10]) > cb)
                 if(*(cache_0 + pixel[9]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else if(*(cache_0 + pixel[7]) < c_b)
              if(*(cache_0 + pixel[1]) > cb)
               if(*(cache_0 + pixel[13]) > cb)
                if(*(cache_0 + pixel[14]) > cb)
                 if(*(cache_0 + pixel[15]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              if(*(cache_0 + pixel[13]) > cb)
               if(*(cache_0 + pixel[14]) > cb)
                if(*(cache_0 + pixel[15]) > cb)
                 if(*(cache_0 + pixel[1]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else if(*(cache_0 + 3) < c_b)
             if(*(cache_0 + pixel[10]) > cb)
              if(*(cache_0 + pixel[11]) > cb)
               if(*(cache_0 + -3) > cb)
                if(*(cache_0 + pixel[13]) > cb)
                 if(*(cache_0 + pixel[14]) > cb)
                  if(*(cache_0 + pixel[1]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    if(*(cache_0 + pixel[7]) > cb)
                     if(*(cache_0 + pixel[9]) > cb)
                      goto success;
                     else
                      continue;
                    else
                     continue;
                  else
                   if(*(cache_0 + pixel[7]) > cb)
                    if(*(cache_0 + pixel[9]) > cb)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             if(*(cache_0 + -3) > cb)
              if(*(cache_0 + pixel[14]) > cb)
               if(*(cache_0 + pixel[10]) > cb)
                if(*(cache_0 + pixel[11]) > cb)
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + pixel[1]) > cb)
                   if(*(cache_0 + pixel[7]) > cb)
                    if(*(cache_0 + pixel[9]) > cb)
                     goto success;
                    else
                     if(*(cache_0 + pixel[15]) > cb)
                      goto success;
                     else
                      continue;
                   else
                    if(*(cache_0 + pixel[15]) > cb)
                     goto success;
                    else
                     continue;
                  else if(*(cache_0 + pixel[1]) < c_b)
                   if(*(cache_0 + pixel[7]) > cb)
                    if(*(cache_0 + pixel[9]) > cb)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                  else
                   if(*(cache_0 + pixel[9]) > cb)
                    if(*(cache_0 + pixel[7]) > cb)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else if(*(cache_0 + pixel[6]) < c_b)
            if(*(cache_0 + -3) > cb)
             if(*(cache_0 + pixel[13]) > cb)
              if(*(cache_0 + pixel[14]) > cb)
               if(*(cache_0 + pixel[15]) > cb)
                if(*(cache_0 + pixel[1]) > cb)
                 if(*(cache_0 + 3) > cb)
                  goto success;
                 else
                  if(*(cache_0 + pixel[10]) > cb)
                   if(*(cache_0 + pixel[11]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                else
                 if(*(cache_0 + pixel[7]) > cb)
                  if(*(cache_0 + pixel[9]) > cb)
                   if(*(cache_0 + pixel[10]) > cb)
                    if(*(cache_0 + pixel[11]) > cb)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            if(*(cache_0 + -3) > cb)
             if(*(cache_0 + pixel[14]) > cb)
              if(*(cache_0 + pixel[15]) > cb)
               if(*(cache_0 + pixel[13]) > cb)
                if(*(cache_0 + pixel[1]) > cb)
                 if(*(cache_0 + 3) > cb)
                  goto success;
                 else
                  if(*(cache_0 + pixel[10]) > cb)
                   if(*(cache_0 + pixel[11]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                else if(*(cache_0 + pixel[1]) < c_b)
                 if(*(cache_0 + pixel[7]) > cb)
                  if(*(cache_0 + pixel[9]) > cb)
                   if(*(cache_0 + pixel[10]) > cb)
                    if(*(cache_0 + pixel[11]) > cb)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 if(*(cache_0 + pixel[7]) > cb)
                  if(*(cache_0 + pixel[10]) > cb)
                   if(*(cache_0 + pixel[11]) > cb)
                    if(*(cache_0 + pixel[9]) > cb)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
          else if(*(cache_0 + pixel[2]) < c_b)
           if(*(cache_0 + -3) > cb)
            if(*(cache_0 + pixel[9]) > cb)
             if(*(cache_0 + pixel[10]) > cb)
              if(*(cache_0 + pixel[11]) > cb)
               if(*(cache_0 + pixel[7]) > cb)
                if(*(cache_0 + pixel[6]) > cb)
                 if(*(cache_0 + 3) > cb)
                  goto success;
                 else
                  if(*(cache_0 + pixel[13]) > cb)
                   if(*(cache_0 + pixel[14]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                else
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[1]) > cb)
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           if(*(cache_0 + pixel[11]) > cb)
            if(*(cache_0 + pixel[10]) > cb)
             if(*(cache_0 + -3) > cb)
              if(*(cache_0 + pixel[9]) > cb)
               if(*(cache_0 + pixel[7]) > cb)
                if(*(cache_0 + pixel[6]) > cb)
                 if(*(cache_0 + 3) > cb)
                  goto success;
                 else if(*(cache_0 + 3) < c_b)
                  if(*(cache_0 + pixel[13]) > cb)
                   if(*(cache_0 + pixel[14]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[13]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                else if(*(cache_0 + pixel[6]) < c_b)
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 if(*(cache_0 + pixel[14]) > cb)
                  if(*(cache_0 + pixel[13]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else if(*(cache_0 + pixel[7]) < c_b)
                if(*(cache_0 + pixel[1]) > cb)
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                if(*(cache_0 + pixel[14]) > cb)
                 if(*(cache_0 + pixel[1]) > cb)
                  if(*(cache_0 + pixel[13]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else if(*(cache_0 + pixel[5]) < c_b)
          if(*(cache_0 + pixel[13]) > cb)
           if(*(cache_0 + pixel[11]) > cb)
            if(*(cache_0 + -3) > cb)
             if(*(cache_0 + pixel[14]) > cb)
              if(*(cache_0 + pixel[15]) > cb)
               if(*(cache_0 + pixel[10]) > cb)
                if(*(cache_0 + pixel[9]) > cb)
                 if(*(cache_0 + pixel[1]) > cb)
                  goto success;
                 else
                  if(*(cache_0 + pixel[7]) > cb)
                   goto success;
                  else
                   continue;
                else
                 if(*(cache_0 + pixel[1]) > cb)
                  if(*(cache_0 + pixel[2]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[1]) > cb)
                 if(*(cache_0 + pixel[2]) > cb)
                  if(*(cache_0 + 3) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          if(*(cache_0 + -3) > cb)
           if(*(cache_0 + pixel[14]) > cb)
            if(*(cache_0 + pixel[11]) > cb)
             if(*(cache_0 + pixel[15]) > cb)
              if(*(cache_0 + pixel[10]) > cb)
               if(*(cache_0 + pixel[13]) > cb)
                if(*(cache_0 + pixel[1]) > cb)
                 if(*(cache_0 + pixel[2]) > cb)
                  goto success;
                 else
                  if(*(cache_0 + pixel[9]) > cb)
                   goto success;
                  else
                   continue;
                else
                 if(*(cache_0 + pixel[7]) > cb)
                  if(*(cache_0 + pixel[9]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else if(*(cache_0 + pixel[10]) < c_b)
               if(*(cache_0 + pixel[1]) > cb)
                if(*(cache_0 + pixel[2]) > cb)
                 if(*(cache_0 + 3) > cb)
                  if(*(cache_0 + pixel[13]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               if(*(cache_0 + 3) > cb)
                if(*(cache_0 + pixel[2]) > cb)
                 if(*(cache_0 + pixel[1]) > cb)
                  if(*(cache_0 + pixel[13]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
        else if(*(cache_0 + pixel[3]) < c_b)
         if(*(cache_0 + -3) > cb)
          if(*(cache_0 + pixel[10]) > cb)
           if(*(cache_0 + pixel[13]) > cb)
            if(*(cache_0 + pixel[9]) > cb)
             if(*(cache_0 + pixel[11]) > cb)
              if(*(cache_0 + pixel[14]) > cb)
               if(*(cache_0 + pixel[15]) > cb)
                if(*(cache_0 + pixel[7]) > cb)
                 goto success;
                else
                 if(*(cache_0 + pixel[1]) > cb)
                  goto success;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[5]) > cb)
                 if(*(cache_0 + pixel[6]) > cb)
                  if(*(cache_0 + pixel[7]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(*(cache_0 + 3) > cb)
                if(*(cache_0 + pixel[5]) > cb)
                 if(*(cache_0 + pixel[6]) > cb)
                  if(*(cache_0 + pixel[7]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          continue;
        else
         if(*(cache_0 + -3) > cb)
          if(*(cache_0 + pixel[10]) > cb)
           if(*(cache_0 + pixel[14]) > cb)
            if(*(cache_0 + pixel[11]) > cb)
             if(*(cache_0 + pixel[13]) > cb)
              if(*(cache_0 + pixel[9]) > cb)
               if(*(cache_0 + pixel[7]) > cb)
                if(*(cache_0 + pixel[15]) > cb)
                 goto success;
                else
                 if(*(cache_0 + pixel[5]) > cb)
                  if(*(cache_0 + pixel[6]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[1]) > cb)
                 if(*(cache_0 + pixel[15]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(*(cache_0 + pixel[14]) < c_b)
            if(*(cache_0 + 3) > cb)
             if(*(cache_0 + pixel[5]) > cb)
              if(*(cache_0 + pixel[6]) > cb)
               if(*(cache_0 + pixel[7]) > cb)
                if(*(cache_0 + pixel[9]) > cb)
                 if(*(cache_0 + pixel[11]) > cb)
                  if(*(cache_0 + pixel[13]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            if(*(cache_0 + 3) > cb)
             if(*(cache_0 + pixel[13]) > cb)
              if(*(cache_0 + pixel[6]) > cb)
               if(*(cache_0 + pixel[11]) > cb)
                if(*(cache_0 + pixel[7]) > cb)
                 if(*(cache_0 + pixel[5]) > cb)
                  if(*(cache_0 + pixel[9]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
          else
           continue;
         else
          continue;
       else if(*(cache_0 + pixel[8]) < c_b)
        if(*(cache_0 + pixel[11]) > cb)
         if(*(cache_0 + pixel[2]) > cb)
          if(*(cache_0 + pixel[15]) > cb)
           if(*(cache_0 + pixel[1]) > cb)
            if(*(cache_0 + pixel[14]) > cb)
             if(*(cache_0 + pixel[13]) > cb)
              if(*(cache_0 + pixel[3]) > cb)
               if(*(cache_0 + -3) > cb)
                if(*(cache_0 + 3) > cb)
                 goto success;
                else
                 if(*(cache_0 + pixel[10]) > cb)
                  goto success;
                 else
                  continue;
               else
                if(*(cache_0 + 3) > cb)
                 if(*(cache_0 + pixel[5]) > cb)
                  if(*(cache_0 + pixel[6]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(*(cache_0 + pixel[9]) > cb)
                if(*(cache_0 + pixel[10]) > cb)
                 if(*(cache_0 + -3) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(*(cache_0 + pixel[3]) > cb)
               if(*(cache_0 + 3) > cb)
                if(*(cache_0 + pixel[5]) > cb)
                 if(*(cache_0 + pixel[6]) > cb)
                  if(*(cache_0 + pixel[7]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else if(*(cache_0 + pixel[2]) < c_b)
          if(*(cache_0 + pixel[1]) < c_b)
           if(*(cache_0 + pixel[3]) < c_b)
            if(*(cache_0 + 3) < c_b)
             if(*(cache_0 + pixel[5]) < c_b)
              if(*(cache_0 + pixel[6]) < c_b)
               if(*(cache_0 + pixel[7]) < c_b)
                if(*(cache_0 + pixel[9]) < c_b)
                 if(*(cache_0 + pixel[10]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          continue;
        else if(*(cache_0 + pixel[11]) < c_b)
         if(*(cache_0 + pixel[6]) > cb)
          if(*(cache_0 + pixel[14]) > cb)
           if(*(cache_0 + pixel[3]) > cb)
            if(*(cache_0 + pixel[1]) > cb)
             if(*(cache_0 + pixel[2]) > cb)
              if(*(cache_0 + 3) > cb)
               if(*(cache_0 + pixel[5]) > cb)
                if(*(cache_0 + pixel[15]) > cb)
                 if(*(cache_0 + pixel[7]) > cb)
                  goto success;
                 else
                  if(*(cache_0 + pixel[13]) > cb)
                   goto success;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else if(*(cache_0 + pixel[6]) < c_b)
          if(*(cache_0 + pixel[10]) > cb)
           if(*(cache_0 + pixel[1]) > cb)
            if(*(cache_0 + pixel[2]) > cb)
             if(*(cache_0 + pixel[3]) > cb)
              if(*(cache_0 + 3) > cb)
               if(*(cache_0 + pixel[5]) > cb)
                if(*(cache_0 + -3) > cb)
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(*(cache_0 + pixel[10]) < c_b)
           if(*(cache_0 + pixel[5]) > cb)
            if(*(cache_0 + pixel[7]) > cb)
             if(*(cache_0 + pixel[1]) > cb)
              if(*(cache_0 + pixel[2]) > cb)
               if(*(cache_0 + pixel[3]) > cb)
                if(*(cache_0 + 3) > cb)
                 if(*(cache_0 + -3) > cb)
                  if(*(cache_0 + pixel[13]) > cb)
                   if(*(cache_0 + pixel[14]) > cb)
                    if(*(cache_0 + pixel[15]) > cb)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else if(*(cache_0 + pixel[7]) < c_b)
             if(*(cache_0 + pixel[14]) > cb)
              if(*(cache_0 + -3) > cb)
               if(*(cache_0 + pixel[1]) > cb)
                if(*(cache_0 + pixel[2]) > cb)
                 if(*(cache_0 + pixel[3]) > cb)
                  if(*(cache_0 + 3) > cb)
                   if(*(cache_0 + pixel[13]) > cb)
                    if(*(cache_0 + pixel[15]) > cb)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(*(cache_0 + pixel[14]) < c_b)
              if(*(cache_0 + pixel[9]) < c_b)
               if(*(cache_0 + -3) < c_b)
                if(*(cache_0 + pixel[13]) < c_b)
                 if(*(cache_0 + pixel[15]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             if(*(cache_0 + -3) > cb)
              if(*(cache_0 + pixel[1]) > cb)
               if(*(cache_0 + pixel[2]) > cb)
                if(*(cache_0 + pixel[3]) > cb)
                 if(*(cache_0 + 3) > cb)
                  if(*(cache_0 + pixel[13]) > cb)
                   if(*(cache_0 + pixel[14]) > cb)
                    if(*(cache_0 + pixel[15]) > cb)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else if(*(cache_0 + pixel[5]) < c_b)
            if(*(cache_0 + -3) > cb)
             if(*(cache_0 + pixel[2]) < c_b)
              if(*(cache_0 + pixel[3]) < c_b)
               if(*(cache_0 + 3) < c_b)
                if(*(cache_0 + pixel[7]) < c_b)
                 if(*(cache_0 + pixel[9]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else if(*(cache_0 + -3) < c_b)
             if(*(cache_0 + pixel[9]) < c_b)
              if(*(cache_0 + 3) > cb)
               if(*(cache_0 + pixel[7]) < c_b)
                if(*(cache_0 + pixel[13]) < c_b)
                 if(*(cache_0 + pixel[14]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(*(cache_0 + 3) < c_b)
               if(*(cache_0 + pixel[7]) < c_b)
                if(*(cache_0 + pixel[13]) < c_b)
                 goto success;
                else
                 if(*(cache_0 + pixel[3]) < c_b)
                  goto success;
                 else
                  continue;
               else
                continue;
              else
               if(*(cache_0 + pixel[14]) < c_b)
                if(*(cache_0 + pixel[13]) < c_b)
                 if(*(cache_0 + pixel[7]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             if(*(cache_0 + pixel[2]) < c_b)
              if(*(cache_0 + pixel[7]) < c_b)
               if(*(cache_0 + pixel[3]) < c_b)
                if(*(cache_0 + pixel[9]) < c_b)
                 if(*(cache_0 + 3) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            if(*(cache_0 + pixel[15]) < c_b)
             if(*(cache_0 + pixel[14]) < c_b)
              if(*(cache_0 + pixel[7]) < c_b)
               if(*(cache_0 + pixel[9]) < c_b)
                if(*(cache_0 + -3) < c_b)
                 if(*(cache_0 + pixel[13]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
          else
           if(*(cache_0 + -3) > cb)
            if(*(cache_0 + pixel[1]) > cb)
             if(*(cache_0 + pixel[2]) > cb)
              if(*(cache_0 + pixel[3]) > cb)
               if(*(cache_0 + 3) > cb)
                if(*(cache_0 + pixel[5]) > cb)
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else
          if(*(cache_0 + -3) > cb)
           if(*(cache_0 + pixel[3]) > cb)
            if(*(cache_0 + pixel[1]) > cb)
             if(*(cache_0 + pixel[2]) > cb)
              if(*(cache_0 + 3) > cb)
               if(*(cache_0 + pixel[5]) > cb)
                if(*(cache_0 + pixel[13]) > cb)
                 if(*(cache_0 + pixel[14]) > cb)
                  if(*(cache_0 + pixel[15]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
        else
         if(*(cache_0 + pixel[3]) > cb)
          if(*(cache_0 + pixel[5]) > cb)
           if(*(cache_0 + pixel[14]) > cb)
            if(*(cache_0 + pixel[15]) > cb)
             if(*(cache_0 + pixel[13]) > cb)
              if(*(cache_0 + pixel[1]) > cb)
               if(*(cache_0 + pixel[2]) > cb)
                if(*(cache_0 + 3) > cb)
                 if(*(cache_0 + pixel[6]) > cb)
                  goto success;
                 else
                  if(*(cache_0 + -3) > cb)
                   goto success;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(*(cache_0 + pixel[13]) < c_b)
              if(*(cache_0 + pixel[6]) > cb)
               if(*(cache_0 + pixel[1]) > cb)
                if(*(cache_0 + pixel[2]) > cb)
                 if(*(cache_0 + 3) > cb)
                  if(*(cache_0 + pixel[7]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              if(*(cache_0 + pixel[7]) > cb)
               if(*(cache_0 + pixel[1]) > cb)
                if(*(cache_0 + pixel[2]) > cb)
                 if(*(cache_0 + 3) > cb)
                  if(*(cache_0 + pixel[6]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else if(*(cache_0 + pixel[3]) < c_b)
          if(*(cache_0 + pixel[1]) < c_b)
           if(*(cache_0 + pixel[10]) < c_b)
            if(*(cache_0 + pixel[2]) < c_b)
             if(*(cache_0 + 3) < c_b)
              if(*(cache_0 + pixel[5]) < c_b)
               if(*(cache_0 + pixel[6]) < c_b)
                if(*(cache_0 + pixel[7]) < c_b)
                 if(*(cache_0 + pixel[9]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          continue;
       else
        if(*(cache_0 + pixel[3]) > cb)
         if(*(cache_0 + pixel[14]) > cb)
          if(*(cache_0 + -3) > cb)
           if(*(cache_0 + pixel[2]) > cb)
            if(*(cache_0 + 3) > cb)
             if(*(cache_0 + pixel[15]) > cb)
              if(*(cache_0 + pixel[1]) > cb)
               if(*(cache_0 + pixel[13]) > cb)
                if(*(cache_0 + pixel[11]) > cb)
                 goto success;
                else
                 if(*(cache_0 + pixel[5]) > cb)
                  goto success;
                 else
                  continue;
               else if(*(cache_0 + pixel[13]) < c_b)
                if(*(cache_0 + pixel[5]) > cb)
                 if(*(cache_0 + pixel[6]) > cb)
                  if(*(cache_0 + pixel[7]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                if(*(cache_0 + pixel[7]) > cb)
                 if(*(cache_0 + pixel[5]) > cb)
                  if(*(cache_0 + pixel[6]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else if(*(cache_0 + 3) < c_b)
             if(*(cache_0 + pixel[1]) > cb)
              if(*(cache_0 + pixel[10]) > cb)
               if(*(cache_0 + pixel[11]) > cb)
                if(*(cache_0 + pixel[13]) > cb)
                 if(*(cache_0 + pixel[15]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             if(*(cache_0 + pixel[10]) > cb)
              if(*(cache_0 + pixel[13]) > cb)
               if(*(cache_0 + pixel[11]) > cb)
                if(*(cache_0 + pixel[15]) > cb)
                 if(*(cache_0 + pixel[1]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            continue;
          else if(*(cache_0 + -3) < c_b)
           if(*(cache_0 + pixel[6]) > cb)
            if(*(cache_0 + pixel[1]) > cb)
             if(*(cache_0 + pixel[2]) > cb)
              if(*(cache_0 + 3) > cb)
               if(*(cache_0 + pixel[5]) > cb)
                if(*(cache_0 + pixel[15]) > cb)
                 if(*(cache_0 + pixel[7]) > cb)
                  goto success;
                 else
                  if(*(cache_0 + pixel[13]) > cb)
                   goto success;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           if(*(cache_0 + pixel[6]) > cb)
            if(*(cache_0 + pixel[2]) > cb)
             if(*(cache_0 + pixel[5]) > cb)
              if(*(cache_0 + pixel[13]) > cb)
               if(*(cache_0 + pixel[15]) > cb)
                if(*(cache_0 + 3) > cb)
                 if(*(cache_0 + pixel[1]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(*(cache_0 + pixel[13]) < c_b)
               if(*(cache_0 + pixel[1]) > cb)
                if(*(cache_0 + 3) > cb)
                 if(*(cache_0 + pixel[7]) > cb)
                  if(*(cache_0 + pixel[15]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               if(*(cache_0 + pixel[7]) > cb)
                if(*(cache_0 + pixel[15]) > cb)
                 if(*(cache_0 + 3) > cb)
                  if(*(cache_0 + pixel[1]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else
          continue;
        else if(*(cache_0 + pixel[3]) < c_b)
         if(*(cache_0 + pixel[2]) > cb)
          if(*(cache_0 + pixel[9]) > cb)
           if(*(cache_0 + pixel[1]) > cb)
            if(*(cache_0 + pixel[10]) > cb)
             if(*(cache_0 + pixel[11]) > cb)
              if(*(cache_0 + -3) > cb)
               if(*(cache_0 + pixel[13]) > cb)
                if(*(cache_0 + pixel[14]) > cb)
                 if(*(cache_0 + pixel[15]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          continue;
        else
         if(*(cache_0 + pixel[9]) > cb)
          if(*(cache_0 + pixel[2]) > cb)
           if(*(cache_0 + -3) > cb)
            if(*(cache_0 + pixel[14]) > cb)
             if(*(cache_0 + pixel[11]) > cb)
              if(*(cache_0 + pixel[13]) > cb)
               if(*(cache_0 + pixel[15]) > cb)
                if(*(cache_0 + pixel[10]) > cb)
                 if(*(cache_0 + pixel[1]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          continue;
      else if(*(cache_0 + pixel[0]) < c_b)
       if(*(cache_0 + pixel[8]) > cb)
        if(*(cache_0 + pixel[2]) > cb)
         if(*(cache_0 + pixel[10]) > cb)
          if(*(cache_0 + pixel[6]) > cb)
           if(*(cache_0 + pixel[7]) > cb)
            if(*(cache_0 + pixel[9]) > cb)
             if(*(cache_0 + pixel[5]) > cb)
              if(*(cache_0 + pixel[11]) > cb)
               if(*(cache_0 + 3) > cb)
                if(*(cache_0 + pixel[3]) > cb)
                 goto success;
                else
                 if(*(cache_0 + -3) > cb)
                  if(*(cache_0 + pixel[13]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + -3) > cb)
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(*(cache_0 + pixel[1]) > cb)
                if(*(cache_0 + pixel[3]) > cb)
                 if(*(cache_0 + 3) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else if(*(cache_0 + pixel[5]) < c_b)
              if(*(cache_0 + pixel[11]) > cb)
               if(*(cache_0 + -3) > cb)
                if(*(cache_0 + pixel[13]) > cb)
                 if(*(cache_0 + pixel[14]) > cb)
                  if(*(cache_0 + pixel[15]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              if(*(cache_0 + pixel[13]) > cb)
               if(*(cache_0 + pixel[11]) > cb)
                if(*(cache_0 + -3) > cb)
                 if(*(cache_0 + pixel[14]) > cb)
                  if(*(cache_0 + pixel[15]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          continue;
        else if(*(cache_0 + pixel[2]) < c_b)
         if(*(cache_0 + pixel[13]) > cb)
          if(*(cache_0 + pixel[6]) > cb)
           if(*(cache_0 + pixel[11]) > cb)
            if(*(cache_0 + pixel[9]) > cb)
             if(*(cache_0 + pixel[7]) > cb)
              if(*(cache_0 + pixel[10]) > cb)
               if(*(cache_0 + pixel[5]) > cb)
                if(*(cache_0 + -3) > cb)
                 if(*(cache_0 + 3) > cb)
                  goto success;
                 else
                  if(*(cache_0 + pixel[14]) > cb)
                   goto success;
                  else
                   continue;
                else
                 continue;
               else
                if(*(cache_0 + pixel[15]) > cb)
                 if(*(cache_0 + -3) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(*(cache_0 + pixel[6]) < c_b)
           if(*(cache_0 + pixel[7]) < c_b)
            if(*(cache_0 + pixel[1]) < c_b)
             if(*(cache_0 + pixel[3]) < c_b)
              if(*(cache_0 + 3) < c_b)
               if(*(cache_0 + pixel[5]) < c_b)
                if(*(cache_0 + pixel[14]) < c_b)
                 if(*(cache_0 + pixel[15]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else if(*(cache_0 + pixel[13]) < c_b)
          if(*(cache_0 + pixel[3]) > cb)
           if(*(cache_0 + pixel[10]) > cb)
            if(*(cache_0 + pixel[7]) > cb)
             if(*(cache_0 + 3) > cb)
              if(*(cache_0 + pixel[5]) > cb)
               if(*(cache_0 + pixel[6]) > cb)
                if(*(cache_0 + pixel[9]) > cb)
                 if(*(cache_0 + pixel[11]) > cb)
                  if(*(cache_0 + -3) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(*(cache_0 + pixel[10]) < c_b)
            if(*(cache_0 + pixel[9]) < c_b)
             if(*(cache_0 + pixel[1]) < c_b)
              if(*(cache_0 + pixel[11]) < c_b)
               if(*(cache_0 + -3) < c_b)
                if(*(cache_0 + pixel[14]) < c_b)
                 if(*(cache_0 + pixel[15]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(*(cache_0 + pixel[3]) < c_b)
           if(*(cache_0 + pixel[15]) < c_b)
            if(*(cache_0 + pixel[1]) < c_b)
             if(*(cache_0 + pixel[5]) > cb)
              if(*(cache_0 + pixel[10]) < c_b)
               if(*(cache_0 + pixel[14]) < c_b)
                if(*(cache_0 + pixel[11]) < c_b)
                 if(*(cache_0 + -3) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               if(*(cache_0 + 3) < c_b)
                if(*(cache_0 + pixel[11]) < c_b)
                 if(*(cache_0 + -3) < c_b)
                  if(*(cache_0 + pixel[14]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else if(*(cache_0 + pixel[5]) < c_b)
              if(*(cache_0 + 3) < c_b)
               if(*(cache_0 + pixel[6]) < c_b)
                if(*(cache_0 + pixel[14]) < c_b)
                 goto success;
                else
                 continue;
               else
                if(*(cache_0 + -3) < c_b)
                 if(*(cache_0 + pixel[14]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
              else
               if(*(cache_0 + pixel[10]) < c_b)
                if(*(cache_0 + pixel[11]) < c_b)
                 if(*(cache_0 + -3) < c_b)
                  if(*(cache_0 + pixel[14]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(*(cache_0 + pixel[11]) < c_b)
               if(*(cache_0 + pixel[10]) > cb)
                if(*(cache_0 + 3) < c_b)
                 if(*(cache_0 + -3) < c_b)
                  if(*(cache_0 + pixel[14]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + pixel[10]) < c_b)
                if(*(cache_0 + pixel[14]) < c_b)
                 if(*(cache_0 + -3) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                if(*(cache_0 + 3) < c_b)
                 if(*(cache_0 + pixel[14]) < c_b)
                  if(*(cache_0 + -3) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           if(*(cache_0 + pixel[9]) < c_b)
            if(*(cache_0 + pixel[11]) < c_b)
             if(*(cache_0 + pixel[1]) < c_b)
              if(*(cache_0 + pixel[10]) < c_b)
               if(*(cache_0 + -3) < c_b)
                if(*(cache_0 + pixel[14]) < c_b)
                 if(*(cache_0 + pixel[15]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else
          if(*(cache_0 + pixel[7]) > cb)
           if(*(cache_0 + pixel[3]) > cb)
            if(*(cache_0 + pixel[10]) > cb)
             if(*(cache_0 + 3) > cb)
              if(*(cache_0 + pixel[5]) > cb)
               if(*(cache_0 + pixel[6]) > cb)
                if(*(cache_0 + pixel[9]) > cb)
                 if(*(cache_0 + pixel[11]) > cb)
                  if(*(cache_0 + -3) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(*(cache_0 + pixel[7]) < c_b)
           if(*(cache_0 + pixel[1]) < c_b)
            if(*(cache_0 + pixel[3]) < c_b)
             if(*(cache_0 + 3) < c_b)
              if(*(cache_0 + pixel[5]) < c_b)
               if(*(cache_0 + pixel[6]) < c_b)
                if(*(cache_0 + pixel[14]) < c_b)
                 if(*(cache_0 + pixel[15]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
        else
         if(*(cache_0 + -3) > cb)
          if(*(cache_0 + pixel[6]) > cb)
           if(*(cache_0 + pixel[11]) > cb)
            if(*(cache_0 + pixel[9]) > cb)
             if(*(cache_0 + pixel[10]) > cb)
              if(*(cache_0 + pixel[13]) > cb)
               if(*(cache_0 + pixel[7]) > cb)
                if(*(cache_0 + pixel[5]) > cb)
                 if(*(cache_0 + 3) > cb)
                  goto success;
                 else
                  if(*(cache_0 + pixel[14]) > cb)
                   goto success;
                  else
                   continue;
                else
                 if(*(cache_0 + pixel[15]) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               if(*(cache_0 + pixel[3]) > cb)
                if(*(cache_0 + 3) > cb)
                 if(*(cache_0 + pixel[5]) > cb)
                  if(*(cache_0 + pixel[7]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          continue;
       else if(*(cache_0 + pixel[8]) < c_b)
        if(*(cache_0 + 3) > cb)
         if(*(cache_0 + -3) < c_b)
          if(*(cache_0 + pixel[10]) < c_b)
           if(*(cache_0 + pixel[14]) < c_b)
            if(*(cache_0 + pixel[15]) < c_b)
             if(*(cache_0 + pixel[13]) < c_b)
              if(*(cache_0 + pixel[1]) < c_b)
               if(*(cache_0 + pixel[11]) < c_b)
                if(*(cache_0 + pixel[9]) > cb)
                 if(*(cache_0 + pixel[2]) < c_b)
                  if(*(cache_0 + pixel[3]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else if(*(cache_0 + pixel[9]) < c_b)
                 goto success;
                else
                 if(*(cache_0 + pixel[3]) < c_b)
                  if(*(cache_0 + pixel[2]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               if(*(cache_0 + pixel[7]) < c_b)
                if(*(cache_0 + pixel[9]) < c_b)
                 if(*(cache_0 + pixel[11]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             if(*(cache_0 + pixel[5]) < c_b)
              if(*(cache_0 + pixel[6]) < c_b)
               if(*(cache_0 + pixel[7]) < c_b)
                if(*(cache_0 + pixel[9]) < c_b)
                 if(*(cache_0 + pixel[11]) < c_b)
                  if(*(cache_0 + pixel[13]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            continue;
          else
           continue;
         else
          continue;
        else if(*(cache_0 + 3) < c_b)
         if(*(cache_0 + pixel[2]) > cb)
          if(*(cache_0 + pixel[10]) < c_b)
           if(*(cache_0 + -3) < c_b)
            if(*(cache_0 + pixel[11]) < c_b)
             if(*(cache_0 + pixel[9]) < c_b)
              if(*(cache_0 + pixel[13]) < c_b)
               if(*(cache_0 + pixel[14]) < c_b)
                if(*(cache_0 + pixel[7]) < c_b)
                 if(*(cache_0 + pixel[15]) > cb)
                  if(*(cache_0 + pixel[5]) < c_b)
                   if(*(cache_0 + pixel[6]) < c_b)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else if(*(cache_0 + pixel[15]) < c_b)
                  goto success;
                 else
                  if(*(cache_0 + pixel[6]) < c_b)
                   if(*(cache_0 + pixel[5]) < c_b)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                else
                 if(*(cache_0 + pixel[1]) < c_b)
                  if(*(cache_0 + pixel[15]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[5]) < c_b)
                 if(*(cache_0 + pixel[6]) < c_b)
                  if(*(cache_0 + pixel[7]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(*(cache_0 + pixel[3]) < c_b)
                if(*(cache_0 + pixel[5]) < c_b)
                 if(*(cache_0 + pixel[6]) < c_b)
                  if(*(cache_0 + pixel[7]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else if(*(cache_0 + pixel[2]) < c_b)
          if(*(cache_0 + pixel[6]) > cb)
           if(*(cache_0 + pixel[13]) < c_b)
            if(*(cache_0 + pixel[14]) < c_b)
             if(*(cache_0 + pixel[15]) < c_b)
              if(*(cache_0 + -3) < c_b)
               if(*(cache_0 + pixel[1]) < c_b)
                if(*(cache_0 + pixel[3]) < c_b)
                 if(*(cache_0 + pixel[11]) < c_b)
                  goto success;
                 else
                  if(*(cache_0 + pixel[5]) < c_b)
                   goto success;
                  else
                   continue;
                else
                 if(*(cache_0 + pixel[9]) < c_b)
                  if(*(cache_0 + pixel[10]) < c_b)
                   if(*(cache_0 + pixel[11]) < c_b)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[7]) < c_b)
                 if(*(cache_0 + pixel[9]) < c_b)
                  if(*(cache_0 + pixel[10]) < c_b)
                   if(*(cache_0 + pixel[11]) < c_b)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(*(cache_0 + pixel[6]) < c_b)
           if(*(cache_0 + pixel[3]) > cb)
            if(*(cache_0 + pixel[9]) < c_b)
             if(*(cache_0 + pixel[10]) < c_b)
              if(*(cache_0 + pixel[11]) < c_b)
               if(*(cache_0 + -3) < c_b)
                if(*(cache_0 + pixel[13]) < c_b)
                 if(*(cache_0 + pixel[7]) < c_b)
                  if(*(cache_0 + pixel[5]) < c_b)
                   goto success;
                  else
                   if(*(cache_0 + pixel[14]) < c_b)
                    if(*(cache_0 + pixel[15]) < c_b)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                 else
                  if(*(cache_0 + pixel[1]) < c_b)
                   if(*(cache_0 + pixel[14]) < c_b)
                    if(*(cache_0 + pixel[15]) < c_b)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(*(cache_0 + pixel[3]) < c_b)
            if(*(cache_0 + pixel[5]) > cb)
             if(*(cache_0 + pixel[11]) < c_b)
              if(*(cache_0 + -3) < c_b)
               if(*(cache_0 + pixel[13]) < c_b)
                if(*(cache_0 + pixel[14]) < c_b)
                 if(*(cache_0 + pixel[15]) < c_b)
                  if(*(cache_0 + pixel[1]) < c_b)
                   goto success;
                  else
                   if(*(cache_0 + pixel[7]) < c_b)
                    if(*(cache_0 + pixel[9]) < c_b)
                     if(*(cache_0 + pixel[10]) < c_b)
                      goto success;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else if(*(cache_0 + pixel[5]) < c_b)
             if(*(cache_0 + pixel[7]) > cb)
              if(*(cache_0 + pixel[1]) < c_b)
               if(*(cache_0 + pixel[13]) < c_b)
                if(*(cache_0 + pixel[14]) < c_b)
                 if(*(cache_0 + pixel[15]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(*(cache_0 + pixel[7]) < c_b)
              if(*(cache_0 + pixel[1]) > cb)
               if(*(cache_0 + pixel[9]) < c_b)
                if(*(cache_0 + pixel[10]) < c_b)
                 if(*(cache_0 + pixel[11]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(*(cache_0 + pixel[1]) < c_b)
               if(*(cache_0 + pixel[9]) < c_b)
                goto success;
               else
                if(*(cache_0 + pixel[15]) < c_b)
                 goto success;
                else
                 continue;
              else
               if(*(cache_0 + pixel[11]) < c_b)
                if(*(cache_0 + pixel[10]) < c_b)
                 if(*(cache_0 + pixel[9]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(*(cache_0 + pixel[13]) < c_b)
               if(*(cache_0 + pixel[15]) < c_b)
                if(*(cache_0 + pixel[14]) < c_b)
                 if(*(cache_0 + pixel[1]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(*(cache_0 + -3) < c_b)
              if(*(cache_0 + pixel[14]) < c_b)
               if(*(cache_0 + pixel[11]) < c_b)
                if(*(cache_0 + pixel[13]) < c_b)
                 if(*(cache_0 + pixel[15]) < c_b)
                  if(*(cache_0 + pixel[1]) > cb)
                   if(*(cache_0 + pixel[7]) < c_b)
                    if(*(cache_0 + pixel[9]) < c_b)
                     if(*(cache_0 + pixel[10]) < c_b)
                      goto success;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else if(*(cache_0 + pixel[1]) < c_b)
                   goto success;
                  else
                   if(*(cache_0 + pixel[9]) < c_b)
                    if(*(cache_0 + pixel[7]) < c_b)
                     if(*(cache_0 + pixel[10]) < c_b)
                      goto success;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            if(*(cache_0 + pixel[11]) < c_b)
             if(*(cache_0 + pixel[13]) < c_b)
              if(*(cache_0 + pixel[10]) < c_b)
               if(*(cache_0 + pixel[9]) < c_b)
                if(*(cache_0 + -3) < c_b)
                 if(*(cache_0 + pixel[7]) > cb)
                  if(*(cache_0 + pixel[1]) < c_b)
                   if(*(cache_0 + pixel[14]) < c_b)
                    if(*(cache_0 + pixel[15]) < c_b)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else if(*(cache_0 + pixel[7]) < c_b)
                  if(*(cache_0 + pixel[5]) < c_b)
                   goto success;
                  else
                   if(*(cache_0 + pixel[14]) < c_b)
                    if(*(cache_0 + pixel[15]) < c_b)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                 else
                  if(*(cache_0 + pixel[15]) < c_b)
                   if(*(cache_0 + pixel[1]) < c_b)
                    if(*(cache_0 + pixel[14]) < c_b)
                     goto success;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
          else
           if(*(cache_0 + -3) < c_b)
            if(*(cache_0 + pixel[14]) < c_b)
             if(*(cache_0 + pixel[15]) < c_b)
              if(*(cache_0 + pixel[13]) < c_b)
               if(*(cache_0 + pixel[11]) > cb)
                if(*(cache_0 + pixel[1]) < c_b)
                 if(*(cache_0 + pixel[3]) < c_b)
                  if(*(cache_0 + pixel[5]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + pixel[11]) < c_b)
                if(*(cache_0 + pixel[1]) > cb)
                 if(*(cache_0 + pixel[7]) < c_b)
                  if(*(cache_0 + pixel[9]) < c_b)
                   if(*(cache_0 + pixel[10]) < c_b)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(*(cache_0 + pixel[1]) < c_b)
                 if(*(cache_0 + pixel[3]) > cb)
                  if(*(cache_0 + pixel[9]) < c_b)
                   if(*(cache_0 + pixel[10]) < c_b)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else if(*(cache_0 + pixel[3]) < c_b)
                  goto success;
                 else
                  if(*(cache_0 + pixel[10]) < c_b)
                   if(*(cache_0 + pixel[9]) < c_b)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                else
                 if(*(cache_0 + pixel[7]) < c_b)
                  if(*(cache_0 + pixel[10]) < c_b)
                   if(*(cache_0 + pixel[9]) < c_b)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[5]) < c_b)
                 if(*(cache_0 + pixel[3]) < c_b)
                  if(*(cache_0 + pixel[1]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else
          if(*(cache_0 + pixel[11]) < c_b)
           if(*(cache_0 + pixel[10]) < c_b)
            if(*(cache_0 + -3) < c_b)
             if(*(cache_0 + pixel[9]) < c_b)
              if(*(cache_0 + pixel[13]) > cb)
               if(*(cache_0 + pixel[3]) < c_b)
                if(*(cache_0 + pixel[5]) < c_b)
                 if(*(cache_0 + pixel[6]) < c_b)
                  if(*(cache_0 + pixel[7]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(*(cache_0 + pixel[13]) < c_b)
               if(*(cache_0 + pixel[7]) < c_b)
                if(*(cache_0 + pixel[6]) < c_b)
                 if(*(cache_0 + pixel[5]) < c_b)
                  goto success;
                 else
                  if(*(cache_0 + pixel[14]) < c_b)
                   if(*(cache_0 + pixel[15]) < c_b)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                else
                 if(*(cache_0 + pixel[14]) < c_b)
                  if(*(cache_0 + pixel[15]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[1]) < c_b)
                 if(*(cache_0 + pixel[14]) < c_b)
                  if(*(cache_0 + pixel[15]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(*(cache_0 + pixel[3]) < c_b)
                if(*(cache_0 + pixel[6]) < c_b)
                 if(*(cache_0 + pixel[7]) < c_b)
                  if(*(cache_0 + pixel[5]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
        else
         if(*(cache_0 + -3) < c_b)
          if(*(cache_0 + pixel[10]) < c_b)
           if(*(cache_0 + pixel[14]) < c_b)
            if(*(cache_0 + pixel[11]) < c_b)
             if(*(cache_0 + pixel[13]) < c_b)
              if(*(cache_0 + pixel[15]) < c_b)
               if(*(cache_0 + pixel[9]) > cb)
                if(*(cache_0 + pixel[1]) < c_b)
                 if(*(cache_0 + pixel[2]) < c_b)
                  if(*(cache_0 + pixel[3]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + pixel[9]) < c_b)
                if(*(cache_0 + pixel[1]) < c_b)
                 goto success;
                else
                 if(*(cache_0 + pixel[7]) < c_b)
                  goto success;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[3]) < c_b)
                 if(*(cache_0 + pixel[2]) < c_b)
                  if(*(cache_0 + pixel[1]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(*(cache_0 + pixel[5]) < c_b)
                if(*(cache_0 + pixel[6]) < c_b)
                 if(*(cache_0 + pixel[7]) < c_b)
                  if(*(cache_0 + pixel[9]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          continue;
       else
        if(*(cache_0 + pixel[2]) < c_b)
         if(*(cache_0 + -3) > cb)
          if(*(cache_0 + pixel[6]) < c_b)
           if(*(cache_0 + pixel[14]) < c_b)
            if(*(cache_0 + pixel[7]) > cb)
             if(*(cache_0 + pixel[1]) < c_b)
              if(*(cache_0 + pixel[3]) < c_b)
               if(*(cache_0 + 3) < c_b)
                if(*(cache_0 + pixel[5]) < c_b)
                 if(*(cache_0 + pixel[13]) < c_b)
                  if(*(cache_0 + pixel[15]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else if(*(cache_0 + pixel[7]) < c_b)
             if(*(cache_0 + 3) < c_b)
              if(*(cache_0 + pixel[5]) < c_b)
               if(*(cache_0 + pixel[1]) < c_b)
                if(*(cache_0 + pixel[3]) < c_b)
                 if(*(cache_0 + pixel[15]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             if(*(cache_0 + pixel[13]) < c_b)
              if(*(cache_0 + pixel[1]) < c_b)
               if(*(cache_0 + pixel[3]) < c_b)
                if(*(cache_0 + 3) < c_b)
                 if(*(cache_0 + pixel[5]) < c_b)
                  if(*(cache_0 + pixel[15]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            continue;
          else
           continue;
         else if(*(cache_0 + -3) < c_b)
          if(*(cache_0 + pixel[3]) > cb)
           if(*(cache_0 + pixel[9]) < c_b)
            if(*(cache_0 + pixel[11]) < c_b)
             if(*(cache_0 + pixel[14]) < c_b)
              if(*(cache_0 + pixel[13]) < c_b)
               if(*(cache_0 + pixel[15]) < c_b)
                if(*(cache_0 + pixel[1]) < c_b)
                 if(*(cache_0 + pixel[10]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(*(cache_0 + pixel[3]) < c_b)
           if(*(cache_0 + pixel[14]) < c_b)
            if(*(cache_0 + 3) > cb)
             if(*(cache_0 + pixel[10]) < c_b)
              if(*(cache_0 + pixel[15]) < c_b)
               if(*(cache_0 + pixel[1]) < c_b)
                if(*(cache_0 + pixel[11]) < c_b)
                 if(*(cache_0 + pixel[13]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else if(*(cache_0 + 3) < c_b)
             if(*(cache_0 + pixel[15]) < c_b)
              if(*(cache_0 + pixel[1]) < c_b)
               if(*(cache_0 + pixel[13]) > cb)
                if(*(cache_0 + pixel[5]) < c_b)
                 if(*(cache_0 + pixel[6]) < c_b)
                  if(*(cache_0 + pixel[7]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + pixel[13]) < c_b)
                if(*(cache_0 + pixel[5]) < c_b)
                 goto success;
                else
                 if(*(cache_0 + pixel[11]) < c_b)
                  goto success;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[7]) < c_b)
                 if(*(cache_0 + pixel[6]) < c_b)
                  if(*(cache_0 + pixel[5]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             if(*(cache_0 + pixel[10]) < c_b)
              if(*(cache_0 + pixel[11]) < c_b)
               if(*(cache_0 + pixel[15]) < c_b)
                if(*(cache_0 + pixel[13]) < c_b)
                 if(*(cache_0 + pixel[1]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            continue;
          else
           if(*(cache_0 + pixel[9]) < c_b)
            if(*(cache_0 + pixel[10]) < c_b)
             if(*(cache_0 + pixel[14]) < c_b)
              if(*(cache_0 + pixel[11]) < c_b)
               if(*(cache_0 + pixel[15]) < c_b)
                if(*(cache_0 + pixel[1]) < c_b)
                 if(*(cache_0 + pixel[13]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else
          if(*(cache_0 + pixel[6]) < c_b)
           if(*(cache_0 + pixel[14]) < c_b)
            if(*(cache_0 + 3) < c_b)
             if(*(cache_0 + pixel[13]) > cb)
              if(*(cache_0 + pixel[7]) < c_b)
               if(*(cache_0 + pixel[3]) < c_b)
                if(*(cache_0 + pixel[1]) < c_b)
                 if(*(cache_0 + pixel[5]) < c_b)
                  if(*(cache_0 + pixel[15]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(*(cache_0 + pixel[13]) < c_b)
              if(*(cache_0 + pixel[5]) < c_b)
               if(*(cache_0 + pixel[15]) < c_b)
                if(*(cache_0 + pixel[1]) < c_b)
                 if(*(cache_0 + pixel[3]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              if(*(cache_0 + pixel[7]) < c_b)
               if(*(cache_0 + pixel[15]) < c_b)
                if(*(cache_0 + pixel[3]) < c_b)
                 if(*(cache_0 + pixel[5]) < c_b)
                  if(*(cache_0 + pixel[1]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
        else
         continue;
      else
       if(*(cache_0 + pixel[8]) > cb)
        if(*(cache_0 + pixel[10]) > cb)
         if(*(cache_0 + 3) > cb)
          if(*(cache_0 + pixel[2]) > cb)
           if(*(cache_0 + pixel[6]) > cb)
            if(*(cache_0 + pixel[7]) > cb)
             if(*(cache_0 + pixel[11]) > cb)
              if(*(cache_0 + pixel[9]) > cb)
               if(*(cache_0 + pixel[5]) > cb)
                if(*(cache_0 + pixel[3]) > cb)
                 goto success;
                else if(*(cache_0 + pixel[3]) < c_b)
                 if(*(cache_0 + -3) > cb)
                  if(*(cache_0 + pixel[13]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + -3) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
               else if(*(cache_0 + pixel[5]) < c_b)
                if(*(cache_0 + -3) > cb)
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                if(*(cache_0 + pixel[15]) > cb)
                 if(*(cache_0 + pixel[14]) > cb)
                  if(*(cache_0 + -3) > cb)
                   if(*(cache_0 + pixel[13]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              if(*(cache_0 + pixel[1]) > cb)
               if(*(cache_0 + pixel[3]) > cb)
                if(*(cache_0 + pixel[5]) > cb)
                 if(*(cache_0 + pixel[9]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else if(*(cache_0 + pixel[2]) < c_b)
           if(*(cache_0 + pixel[11]) > cb)
            if(*(cache_0 + -3) > cb)
             if(*(cache_0 + pixel[9]) > cb)
              if(*(cache_0 + pixel[6]) > cb)
               if(*(cache_0 + pixel[7]) > cb)
                if(*(cache_0 + pixel[13]) > cb)
                 if(*(cache_0 + pixel[5]) > cb)
                  goto success;
                 else
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                else
                 if(*(cache_0 + pixel[3]) > cb)
                  if(*(cache_0 + pixel[5]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           if(*(cache_0 + -3) > cb)
            if(*(cache_0 + pixel[6]) > cb)
             if(*(cache_0 + pixel[11]) > cb)
              if(*(cache_0 + pixel[13]) > cb)
               if(*(cache_0 + pixel[7]) > cb)
                if(*(cache_0 + pixel[9]) > cb)
                 if(*(cache_0 + pixel[5]) > cb)
                  goto success;
                 else if(*(cache_0 + pixel[5]) < c_b)
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                 else
                  if(*(cache_0 + pixel[15]) > cb)
                   if(*(cache_0 + pixel[14]) > cb)
                    goto success;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else if(*(cache_0 + pixel[13]) < c_b)
               if(*(cache_0 + pixel[3]) > cb)
                if(*(cache_0 + pixel[5]) > cb)
                 if(*(cache_0 + pixel[7]) > cb)
                  if(*(cache_0 + pixel[9]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               if(*(cache_0 + pixel[3]) > cb)
                if(*(cache_0 + pixel[7]) > cb)
                 if(*(cache_0 + pixel[9]) > cb)
                  if(*(cache_0 + pixel[5]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else if(*(cache_0 + 3) < c_b)
          if(*(cache_0 + pixel[6]) > cb)
           if(*(cache_0 + pixel[14]) > cb)
            if(*(cache_0 + pixel[13]) > cb)
             if(*(cache_0 + pixel[7]) > cb)
              if(*(cache_0 + pixel[15]) > cb)
               if(*(cache_0 + pixel[9]) > cb)
                if(*(cache_0 + pixel[11]) > cb)
                 if(*(cache_0 + -3) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               if(*(cache_0 + pixel[5]) > cb)
                if(*(cache_0 + pixel[9]) > cb)
                 if(*(cache_0 + pixel[11]) > cb)
                  if(*(cache_0 + -3) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          if(*(cache_0 + pixel[14]) > cb)
           if(*(cache_0 + pixel[6]) > cb)
            if(*(cache_0 + -3) > cb)
             if(*(cache_0 + pixel[5]) > cb)
              if(*(cache_0 + pixel[11]) > cb)
               if(*(cache_0 + pixel[9]) > cb)
                if(*(cache_0 + pixel[7]) > cb)
                 if(*(cache_0 + pixel[13]) > cb)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(*(cache_0 + pixel[5]) < c_b)
              if(*(cache_0 + pixel[15]) > cb)
               if(*(cache_0 + pixel[7]) > cb)
                if(*(cache_0 + pixel[9]) > cb)
                 if(*(cache_0 + pixel[11]) > cb)
                  if(*(cache_0 + pixel[13]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              if(*(cache_0 + pixel[15]) > cb)
               if(*(cache_0 + pixel[11]) > cb)
                if(*(cache_0 + pixel[9]) > cb)
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + pixel[7]) > cb)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
        else
         continue;
       else if(*(cache_0 + pixel[8]) < c_b)
        if(*(cache_0 + pixel[10]) < c_b)
         if(*(cache_0 + 3) > cb)
          if(*(cache_0 + pixel[14]) < c_b)
           if(*(cache_0 + pixel[6]) < c_b)
            if(*(cache_0 + -3) < c_b)
             if(*(cache_0 + pixel[9]) < c_b)
              if(*(cache_0 + pixel[11]) < c_b)
               if(*(cache_0 + pixel[15]) < c_b)
                if(*(cache_0 + pixel[13]) < c_b)
                 if(*(cache_0 + pixel[7]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                if(*(cache_0 + pixel[5]) < c_b)
                 if(*(cache_0 + pixel[7]) < c_b)
                  if(*(cache_0 + pixel[13]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else if(*(cache_0 + 3) < c_b)
          if(*(cache_0 + pixel[6]) < c_b)
           if(*(cache_0 + -3) > cb)
            if(*(cache_0 + pixel[2]) < c_b)
             if(*(cache_0 + pixel[1]) > cb)
              if(*(cache_0 + pixel[3]) < c_b)
               if(*(cache_0 + pixel[5]) < c_b)
                if(*(cache_0 + pixel[7]) < c_b)
                 if(*(cache_0 + pixel[9]) < c_b)
                  if(*(cache_0 + pixel[11]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(*(cache_0 + pixel[1]) < c_b)
              if(*(cache_0 + pixel[5]) < c_b)
               if(*(cache_0 + pixel[9]) < c_b)
                if(*(cache_0 + pixel[3]) < c_b)
                 if(*(cache_0 + pixel[7]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              if(*(cache_0 + pixel[11]) < c_b)
               if(*(cache_0 + pixel[3]) < c_b)
                if(*(cache_0 + pixel[5]) < c_b)
                 if(*(cache_0 + pixel[7]) < c_b)
                  if(*(cache_0 + pixel[9]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else if(*(cache_0 + -3) < c_b)
            if(*(cache_0 + pixel[7]) < c_b)
             if(*(cache_0 + pixel[11]) > cb)
              if(*(cache_0 + pixel[1]) < c_b)
               if(*(cache_0 + pixel[2]) < c_b)
                if(*(cache_0 + pixel[3]) < c_b)
                 if(*(cache_0 + pixel[5]) < c_b)
                  if(*(cache_0 + pixel[9]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(*(cache_0 + pixel[11]) < c_b)
              if(*(cache_0 + pixel[9]) < c_b)
               if(*(cache_0 + pixel[5]) > cb)
                if(*(cache_0 + pixel[13]) < c_b)
                 if(*(cache_0 + pixel[14]) < c_b)
                  if(*(cache_0 + pixel[15]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + pixel[5]) < c_b)
                if(*(cache_0 + pixel[13]) < c_b)
                 goto success;
                else
                 if(*(cache_0 + pixel[3]) < c_b)
                  goto success;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[15]) < c_b)
                 if(*(cache_0 + pixel[14]) < c_b)
                  if(*(cache_0 + pixel[13]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              if(*(cache_0 + pixel[1]) < c_b)
               if(*(cache_0 + pixel[2]) < c_b)
                if(*(cache_0 + pixel[9]) < c_b)
                 if(*(cache_0 + pixel[3]) < c_b)
                  if(*(cache_0 + pixel[5]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            if(*(cache_0 + pixel[2]) < c_b)
             if(*(cache_0 + pixel[1]) < c_b)
              if(*(cache_0 + pixel[3]) < c_b)
               if(*(cache_0 + pixel[7]) < c_b)
                if(*(cache_0 + pixel[9]) < c_b)
                 if(*(cache_0 + pixel[5]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              if(*(cache_0 + pixel[11]) < c_b)
               if(*(cache_0 + pixel[3]) < c_b)
                if(*(cache_0 + pixel[5]) < c_b)
                 if(*(cache_0 + pixel[7]) < c_b)
                  if(*(cache_0 + pixel[9]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
          else
           continue;
         else
          if(*(cache_0 + pixel[14]) < c_b)
           if(*(cache_0 + pixel[6]) < c_b)
            if(*(cache_0 + -3) < c_b)
             if(*(cache_0 + pixel[5]) > cb)
              if(*(cache_0 + pixel[9]) < c_b)
               if(*(cache_0 + pixel[7]) < c_b)
                if(*(cache_0 + pixel[11]) < c_b)
                 if(*(cache_0 + pixel[13]) < c_b)
                  if(*(cache_0 + pixel[15]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(*(cache_0 + pixel[5]) < c_b)
              if(*(cache_0 + pixel[13]) < c_b)
               if(*(cache_0 + pixel[11]) < c_b)
                if(*(cache_0 + pixel[7]) < c_b)
                 if(*(cache_0 + pixel[9]) < c_b)
                  goto success;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              if(*(cache_0 + pixel[15]) < c_b)
               if(*(cache_0 + pixel[13]) < c_b)
                if(*(cache_0 + pixel[7]) < c_b)
                 if(*(cache_0 + pixel[9]) < c_b)
                  if(*(cache_0 + pixel[11]) < c_b)
                   goto success;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
        else
         continue;
       else
        continue;

      success:
        corners.push_back(Eigen::Vector2d(cache_0-line_min, y));
    }
  }
}

void nonmax_suppression(std::vector<Eigen::Vector2d> &corners, std::vector<int> &scores, std::vector<Eigen::Vector2d> &nonmax_corners) {
  nonmax_corners.clear();
  nonmax_corners.reserve(corners.size());

  if(corners.size() < 1)
    return;

  // Find where each row begins
  // (the corners are output in raster scan order). A beginning of -1 signifies
  // that there are no corners on that row.
  int last_row = corners.back()(1);
  std::vector<int> row_start(last_row + 1, -1);

  int prev_row = -1;
  for(unsigned int i=0; i< corners.size(); i++)
    if(corners[i](1) != prev_row)
    {
      row_start[corners[i](1)] = i;
      prev_row = corners[i](1);
    }
  
  
  //Point above points (roughly) to the pixel above the one of interest, if there
  //is a feature there.
  int point_above = 0;
  int point_below = 0;
  
  const int sz = (int)corners.size(); 
  
  for(int i=0; i < sz; i++)
  {
    int score = scores[i];
    Eigen::Vector2d pos(corners[i](0), corners[i](1));
      
    //Check left 
    if(i > 0)
      if(corners[i-1](0) == (pos(0)-1) && corners[i-1](1) == pos(1) && scores[i-1] > score)
        continue;
      
    //Check right
    if(i < (sz - 1))
      if(corners[i+1](0) == (pos(0)+1) && corners[i-1](1) == pos(1) && scores[i+1] > score)
        continue;
      
    //Check above (if there is a valid row above)
    if(pos(1) != 0 && row_start[pos(1) - 1] != -1) 
    {
      //Make sure that current point_above is one
      //row above.
      if(corners[point_above](1) < pos(1) - 1)
        point_above = row_start[pos(1)-1];
      
      //Make point_above point to the first of the pixels above the current point,
      //if it exists.
      for(; corners[point_above](1) < pos(1) && corners[point_above](0) < pos(0) - 1; point_above++)
      {}
      
      
      for(int i=point_above; corners[i](1) < pos(1) && corners[i](0) <= pos(0) + 1; i++)
      {
        int x = corners[i](0);
        if( (x == pos(0) - 1 || x ==pos(0) || x == pos(0)+1) && scores[i] > score)
          goto cont;
      }
      
    }
      
    //Check below (if there is anything below)
    if(pos(1) != last_row && row_start[pos(1) + 1] != -1 && point_below < sz) //Nothing below
    {
      if(corners[point_below](1) < pos(1) + 1)
        point_below = row_start[pos(1)+1];
      
      // Make point below point to one of the pixels belowthe current point, if it
      // exists.
      for(; point_below < sz && corners[point_below](1) == pos(1)+1 && corners[point_below](0) < pos(0) - 1; point_below++)
      {}

      for(int i=point_below; i < sz && corners[i](1) == pos(1)+1 && corners[i](0) <= pos(0) + 1; i++)
      {
        int x = corners[i](0);
        if( (x == pos(0) - 1 || x ==pos(0) || x == pos(0)+1) && scores[i] > score)
          goto cont;
      }
    }
      
    //nonmax_corners.push_back(Eigen::Vector2d(corners[i](0), corners[i](1)),scores[i]));
    nonmax_corners.push_back(Eigen::Vector2d(corners[i](0), corners[i](1)));
      
    cont:
      ;
  }
}

int old_style_corner_score(cv::Mat &im, Eigen::Vector2d c, const int *pointer_dir, int barrier)
{
  //The score for a positive feature is sum of the difference between the pixels
  //and the barrier if the difference is positive. Negative is similar.
  //The score is the max of those two.
  //
  // B = {x | x = points on the Bresenham circle around c}
  // Sp = { I(x) - t | x E B , I(x) - t > 0 }
  // Sn = { t - I(x) | x E B, t - I(x) > 0}
  //
  // Score = max sum(Sp), sum(Sn)

  const unsigned char* imp = im.data + (int)c(1)*im.step + (int)c(0);
  
  int cb = *imp + barrier;
  int c_b = *imp - barrier;
  int sp=0, sn = 0;

  for(int i=0; i<16; i++)
  {
    int p = *(imp + pointer_dir[i]);

    if(p > cb)
      sp += p-cb;
    else if(p < c_b)
      sn += c_b-p;
  }
  
  if(sp > sn)
    return sp;
  else 
    return sn;
}

void compute_fast_score_old(cv::Mat &src, std::vector<Eigen::Vector2d> &corners, int barrier, std::vector<int> &scores)
{
  int pointer_dir[16] = { /*TODO: Es posible que aquí sea src.cols en lugar de src.step*/
    0 + src.step * 3,
    1 + src.step * 3,
    2 + src.step * 2,
    3 + src.step * 1,
    3 + src.step * 0,
    3 + src.step * -1,
    2 + src.step * -2,
    1 + src.step * -3,
    0 + src.step * -3,
    -1 + src.step * -3,
    -2 + src.step * -2,
    -3 + src.step * -1,
    -3 + src.step * 0,
    -3 + src.step * 1,
    -2 + src.step * 2,
    -1 + src.step * 3,
  };

  scores.resize(corners.size());

  for(unsigned int i=0; i < corners.size(); i++)
    scores[i] = old_style_corner_score(src, corners[i], pointer_dir, barrier);
}



CVAPI(void) fast_nonmax(cv::Mat &src, std::vector<Eigen::Vector2d> &corners, int barrier, std::vector<Eigen::Vector2d> &max_corners)
{
  std::vector<int> scores;
  compute_fast_score_old(src, corners, barrier, scores);
  nonmax_suppression(corners, scores, max_corners);
}

