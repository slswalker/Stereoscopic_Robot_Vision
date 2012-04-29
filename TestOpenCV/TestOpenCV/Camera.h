//
//  Camera.h
//  TestOpenCV
//
//  Created by Sam Walker on 3/27/12.
//  Copyright (c) 2012 Sam Walker. All rights reserved.
//

#ifndef TestOpenCV_Camera_h
#define TestOpenCV_Camera_h

//#include "opencv2/core/core_c.h"
//#include "opencv2/core/core.hpp"
//#include "opencv2/imgproc/imgproc_c.h"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/flann/flann.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/legacy/compat.hpp"
//
//#include "cxmisc.h"
//#include "highgui.h"
////#include "cvaux.h"
//#include <vector>
//#include <string>
//#include <algorithm>
//#include <stdio.h>
//#include <ctype.h>
#include "Constants.h"
//#include "Constants.h"


class Camera
{
private:
    int mx = 9;
    int my = 6;
    int mWidth = 2.5;
    const char* videoLeft = "left";
    const char* videoRight = "right";
    CvCapture* cameraLeft;
    CvCapture* cameraRight;
    IplImage* imgArr[50];
    int active = 0;
    CvStereoBMState *mBMState;
    const char* imagesFile;
    
    CvSize imageSize;
    
    CvMat *Q;
    CvMat *mx1;
    CvMat *my1;
    CvMat *mx2;
    CvMat *my2;
    
public:
    
    Camera();
    
    Camera(int px, int py, int width);
    
    void setDefaultBMState();

    void loadImages(const char* file);
    
    void loadMats();
    void saveMats();
    
    void takePictures();
    
    void setImageFile(const char* file);
    
    void setCameras(int x, int y);
    
    void stereoCalibFile(const char* file, int useUncalibrated, int save);
    void stereoCalibPictures(int useUncalibrated, int save);
    
    void showCamera();
    
    void showDepthMap();
    
    float getDepthMap();
    
    void stereoCalibrateFile();
    void stereoCalibrateArray();
    
    float getObjects(float radius);
    
    
    
    
};

#endif
