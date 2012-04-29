//
//  Camera.cpp
//  TestOpenCV
//
//  Created by Sam Walker on 3/27/12.
//  Copyright (c) 2012 Sam Walker. All rights reserved.
//

#include <iostream>
#include <fstream>
#include "Timer.h"
#include "Camera.h"

Camera::Camera()
{
    setDefaultBMState();
    setCameras(0, 2);
}

Camera::Camera(int px, int py, int width)

{
    mx = px;
    my = py;
    mWidth = width;
    setDefaultBMState();   
    setCameras(0, 2);
}

void Camera::loadImages(const char* file)
{
    std::ifstream fin(file);
    if(!fin.is_open())
        return;
    
    std::string tempStr;
    
    for (int x = 0; x < sizeof(imgArr)/(sizeof(IplImage*)); x++) {
        fin >> tempStr;
        imgArr[x] = cvLoadImage( tempStr.c_str(), 0 );
    }
    fin.close();
}

void Camera::loadMats()
{
    Q   = (CvMat *)cvLoad("Q.xml"  ,NULL,NULL,NULL);
    mx1 = (CvMat *)cvLoad("mx1.xml",NULL,NULL,NULL);
    my1 = (CvMat *)cvLoad("my1.xml",NULL,NULL,NULL);
    mx2 = (CvMat *)cvLoad("mx2.xml",NULL,NULL,NULL);
    my2 = (CvMat *)cvLoad("my2.xml",NULL,NULL,NULL);
}

void Camera::saveMats()
{
    cvSave("Q.xml",Q);
    cvSave("mx1.xml",mx1);
    cvSave("my1.xml",my1);
    cvSave("mx2.xml",mx2);
    cvSave("my2.xml",my2);
}

void Camera::takePictures()
{
    printf("Ready to take pictures.\n");
    Timer* timer = new Timer();
    cvResizeWindow( videoLeft, 640, 480 );
    cvResizeWindow( videoRight, 640, 480 );
    cvShowImage(videoLeft, cvQueryFrame(cameraLeft));
    cvShowImage(videoRight, cvQueryFrame(cameraRight));
    char key = cvWaitKey(0);
    int x = 0;
    timer->start();
    printf("Taking pictures. Be still.\n");
    printf("Num Pics: %d\n",PICTURES);
    while (timer->elapsedTime() < 5);
    timer->start();
    while (x < PICTURES) {
        while (timer->elapsedTime() < 8) {
            key = cvWaitKey(27);
            if (key == 'q') {
                break;
            }
            cvResizeWindow( videoLeft, 640, 480 );
            cvResizeWindow( videoRight, 640, 480 );
            cvShowImage(videoLeft, cvQueryFrame(cameraLeft));
            cvShowImage(videoRight, cvQueryFrame(cameraRight));
        }
        printf("Num Pics: %d\n",x);
        imgArr[x++] = cvCloneImage(cvQueryFrame(cameraLeft));
        imgArr[x++] = cvCloneImage(cvQueryFrame(cameraRight));
        printf("Took a set of Pictures!.\n");
        timer->start();
    }
    delete timer;
    printf("Done taking photos.\n");
}


void Camera::setDefaultBMState()
{
    mBMState->preFilterSize=41;
    mBMState->preFilterCap=31;
    mBMState->SADWindowSize=41;
    mBMState->minDisparity=-64;
    mBMState->numberOfDisparities=128;
    mBMState->textureThreshold=10;
    mBMState->uniquenessRatio=15;
}

void Camera::setImageFile(const char *file)
{
    imagesFile = file;
}

// preferably pass in (0, 2) if computer has a camera. Else (0,1)
void Camera::setCameras(int x, int y)
{
    cameraLeft = cvCreateCameraCapture(x);
    cameraRight = cvCreateCameraCapture(y);
}

void Camera::stereoCalibFile(const char* file, int useUncalibrated, int save)
{    
    using namespace std;
    using namespace cv;
    int displayCorners = 0;
    int showUndistorted = 1;
    bool isVerticalStereo = false;//OpenCV can handle left-right
    //or up-down camera arrangements
    const int maxScale = 1;
    const float squareSize = 1.f; //Set this to your actual square size
    FILE* f = fopen(file, "rt");
    if( !f )
    {
        fprintf(stderr, "can not open file %s\n", file );
        return;
    }
    
    vector<string> imageNames[2];
    
    int i, j, lr, nframes, n = mx*my, activeVar = 0, N = 0;
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<CvPoint2D32f> temp(n);
    CvSize imageSize = {0,0};
    // ARRAY AND VECTOR STORAGE:
    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];
    double Q[4][4];
    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
    CvMat _R = cvMat(3, 3, CV_64F, R );
    CvMat _T = cvMat(3, 1, CV_64F, T );
    CvMat _E = cvMat(3, 3, CV_64F, E );
    CvMat _F = cvMat(3, 3, CV_64F, F );
    CvMat _Q = cvMat(4,4, CV_64F, Q);
    if( displayCorners )
        cvNamedWindow( "corners", 1 );
    // READ IN THE LIST OF CHESSBOARDS:
    
    for(i=0;i< PICTURES * 2;i++)
    {
        
        char buf[1024];
        int count = 0, result=0;
        if( !fgets( buf, sizeof(buf)-3, f ))
            break;
        size_t len = strlen(buf);
        while( len > 0 && isspace(buf[len-1]))
            buf[--len] = '\0';
        if( buf[0] == '#')
            continue;
        lr = i % 2;
        IplImage* img = cvLoadImage( buf, 0 );
        imageNames[lr].push_back(buf);
        vector<CvPoint2D32f>& pts = points[i%2];
        if( !img )
            break;
        imageSize = cvGetSize(img);
        //FIND CHESSBOARDS AND CORNERS THEREIN:
        for( int s = 1; s <= maxScale; s++ )
        {
            IplImage* timg = img;
            if( s > 1 )
            {
                timg = cvCreateImage(cvSize(img->width*s,img->height*s),
                                     img->depth, img->nChannels );
                cvResize( img, timg, CV_INTER_CUBIC );
            }
            result = cvFindChessboardCorners( timg, cvSize(mx, my),
                                             &temp[0], &count,
                                             CV_CALIB_CB_ADAPTIVE_THRESH |
                                             CV_CALIB_CB_NORMALIZE_IMAGE);
            if( timg != img )
                cvReleaseImage( &timg );
            if( result || s == maxScale )
                for( j = 0; j < count; j++ )
                {
                    temp[j].x /= s;
                    temp[j].y /= s;
                }
            if( result )
                break;
        }
        if( displayCorners )
        {
            printf("%s\n", buf);
            IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
            cvCvtColor( img, cimg, CV_GRAY2BGR );
            cvDrawChessboardCorners( cimg, cvSize(mx, my), &temp[0],
                                    count, result );
            cvShowImage( "corners", cimg );
            cvReleaseImage( &cimg );
            if( cvWaitKey(0) == 27 ) //Allow ESC to quit
                exit(-1);
        }
        else
            putchar('.');
        N = pts.size();
        pts.resize(N + n, cvPoint2D32f(0,0));
        active[lr].push_back((uchar)result);
        activeVar++;
        //assert( result != 0 );
        if( result )
        {
            //Calibration will suffer without subpixel interpolation
            cvFindCornerSubPix( img, &temp[0], count,
                               cvSize(11, 11), cvSize(-1,-1),
                               cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                              30, 0.01) );
            copy( temp.begin(), temp.end(), pts.begin() + N );
        }
        cvReleaseImage( &img );
    }
    fclose(f);
    nframes = active[0].size();//Number of good chessboads found
    //    nframes = activeVar;
    
    printf("\n");
    // HARVEST CHESSBOARD 3D OBJECT POINT LIST:
    objectPoints.resize(nframes*n);
    for( i = 0; i < my; i++ )
        for( j = 0; j < mx; j++ )
            objectPoints[i*mx + j] =
            cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < nframes; i++ )
        copy( objectPoints.begin(), objectPoints.begin() + n,
             objectPoints.begin() + i*n );
    npoints.resize(nframes,n);
    N = nframes*n;
    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
    cvSetIdentity(&_M1);
    cvSetIdentity(&_M2);
    cvZero(&_D1);
    cvZero(&_D2);
    
    // CALIBRATE THE STEREO CAMERAS
    printf("Running stereo calibration ...");
    fflush(stdout);
    cvStereoCalibrate( &_objectPoints, &_imagePoints1,
                      &_imagePoints2, &_npoints,
                      &_M1, &_D1, &_M2, &_D2,
                      imageSize, &_R, &_T, &_E, &_F,
                      cvTermCriteria(CV_TERMCRIT_ITER+
                                     CV_TERMCRIT_EPS, 100, 1e-5),
                      CV_CALIB_FIX_ASPECT_RATIO +
                      CV_CALIB_ZERO_TANGENT_DIST +
                      CV_CALIB_SAME_FOCAL_LENGTH );
    printf(" done\n");
    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    vector<CvPoint3D32f> lines[2];
    points[0].resize(N);
    points[1].resize(N);
    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    lines[0].resize(N);
    lines[1].resize(N);
    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
    //Always work in undistorted space
    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
                      &_M1, &_D1, 0, &_M1 );
    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
                      &_M2, &_D2, 0, &_M2 );
    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
    double avgErr = 0;
    for( i = 0; i < N; i++ )
    {
        double err = fabs(points[0][i].x*lines[1][i].x +
                          points[0][i].y*lines[1][i].y + lines[1][i].z)
        + fabs(points[1][i].x*lines[0][i].x +
               points[1][i].y*lines[0][i].y + lines[0][i].z);
        avgErr += err;
    }
    printf( "avg err = %g\n", avgErr/(nframes*n) );
    //COMPUTE AND DISPLAY RECTIFICATION
    if( showUndistorted )
    {
        CvMat* mx1 = cvCreateMat( imageSize.height,
                                 imageSize.width, CV_32F );
        CvMat* my1 = cvCreateMat( imageSize.height,
                                 imageSize.width, CV_32F );
        CvMat* mx2 = cvCreateMat( imageSize.height,
                                 
                                 imageSize.width, CV_32F );
        CvMat* my2 = cvCreateMat( imageSize.height,
                                 imageSize.width, CV_32F );
        
        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
        CvMat _R2 = cvMat(3, 3, CV_64F, R2);
        // IF BY CALIBRATED (BOUGUET'S METHOD)
        if( useUncalibrated == 0 )
        {
            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
            cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
                            &_R, &_T,
                            &_R1, &_R2, &_P1, &_P2, &_Q,
                            0/*CV_CALIB_ZERO_DISPARITY*/ );
            isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
            //Precompute maps for cvRemap()
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
            cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
            
            if(save)
            {
                //Save parameters
                cvSave("M1.xml",&_M1);
                cvSave("D1.xml",&_D1);
                cvSave("R1.xml",&_R1);
                cvSave("P1.xml",&_P1);
                cvSave("M2.xml",&_M2);
                cvSave("D2.xml",&_D2);
                cvSave("R2.xml",&_R2);
                cvSave("P2.xml",&_P2);
                cvSave("Q.xml",&_Q);
                cvSave("mx1.xml",mx1);
                cvSave("my1.xml",my1);
                cvSave("mx2.xml",mx2);
                cvSave("my2.xml",my2);
            }
        }
    }
}

void Camera::stereoCalibPictures(int useUncalibrated, int save)
{    
    using namespace std;
    using namespace cv;
    int displayCorners = 0;
    int showUndistorted = 1;
    bool isVerticalStereo = false;//OpenCV can handle left-right
    //or up-down camera arrangements
    const int maxScale = 1;
    const float squareSize = 1.f; //Set this to your actual square size
    int i, j, lr, nframes, n = mx*my, activeVar = 0, N = 0;
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<CvPoint2D32f> temp(n);
    CvSize imageSize = {0,0};
    // ARRAY AND VECTOR STORAGE:
    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];
    double Q[4][4];
    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
    CvMat _R = cvMat(3, 3, CV_64F, R );
    CvMat _T = cvMat(3, 1, CV_64F, T );
    CvMat _E = cvMat(3, 3, CV_64F, E );
    CvMat _F = cvMat(3, 3, CV_64F, F );
    CvMat _Q = cvMat(4,4, CV_64F, Q);
    
    if( displayCorners )
        cvNamedWindow( "corners", 1 );
    // READ IN THE LIST OF CHESSBOARDS:
    for(i=0;i< PICTURES;i++)
    {
        
        int count = 0, result=0;
        lr = i % 2;
        vector<CvPoint2D32f>& pts = points[i%2];
        IplImage* img = imgArr[i];
        if( !img )
            break;
        imageSize = cvGetSize(img);
        //FIND CHESSBOARDS AND CORNERS THEREIN:
        for( int s = 1; s <= maxScale; s++ )
        {
            IplImage* timg = img;
            if( s > 1 )
            {
                timg = cvCreateImage(cvSize(img->width*s,img->height*s),
                                     img->depth, img->nChannels );
                cvResize( img, timg, CV_INTER_CUBIC );
            }
            result = cvFindChessboardCorners( timg, cvSize(mx, my),
                                             &temp[0], &count,
                                             CV_CALIB_CB_ADAPTIVE_THRESH |
                                             CV_CALIB_CB_NORMALIZE_IMAGE);
            if( timg != img )
                cvReleaseImage( &timg );
            if( result || s == maxScale )
                for( j = 0; j < count; j++ )
                {
                    temp[j].x /= s;
                    temp[j].y /= s;
                }
            if( result )
                break;
        }
        if( displayCorners )
        {
            IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
            cvCvtColor( img, cimg, CV_GRAY2BGR );
            cvDrawChessboardCorners( cimg, cvSize(mx, my), &temp[0],
                                    count, result );
            cvShowImage( "corners", cimg );
            cvReleaseImage( &cimg );
            if( cvWaitKey(0) == 27 ) //Allow ESC to quit
                exit(-1);
        }
        N = pts.size();
        pts.resize(N + n, cvPoint2D32f(0,0));
        activeVar++;
        //assert( result != 0 );
        
        if( result )
        {
            IplImage* vimg = cvCreateImage(cvSize(img->width,img->height),
                                           img->depth, 1  );
            cvFindCornerSubPix( vimg, &temp[0], count,
                               cvSize(11, 11), cvSize(-1,-1),
                               cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                              30, 0.01) );
            //Calibration will suffer without subpixel interpolation
            //            cvFindCornerSubPix( img, &temp[0], count,
            //                               cvSize(11, 11), cvSize(-1,-1),
            //                               cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
            //                                              30, 0.01) );
            copy( temp.begin(), temp.end(), pts.begin() + N );
        }
        cvReleaseImage( &img );
    }
    nframes = activeVar;//Number of good chessboads found
    //    nframes = activeVar;
    
    printf("\n");
    // HARVEST CHESSBOARD 3D OBJECT POINT LIST:
    objectPoints.resize(nframes*n);
    for( i = 0; i < my; i++ )
        for( j = 0; j < mx; j++ )
            objectPoints[i*mx + j] =
            cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < nframes; i++ )
        copy( objectPoints.begin(), objectPoints.begin() + n,
             objectPoints.begin() + i*n );
    npoints.resize(nframes,n);
    N = nframes*n;
    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
    cvSetIdentity(&_M1);
    cvSetIdentity(&_M2);
    cvZero(&_D1);
    cvZero(&_D2);
    
    // CALIBRATE THE STEREO CAMERAS
    printf("Running stereo calibration ...");
    fflush(stdout);
    cvStereoCalibrate( &_objectPoints, &_imagePoints1,
                      &_imagePoints2, &_npoints,
                      &_M1, &_D1, &_M2, &_D2,
                      imageSize, &_R, &_T, &_E, &_F,
                      cvTermCriteria(CV_TERMCRIT_ITER+
                                     CV_TERMCRIT_EPS, 100, 1e-5),
                      CV_CALIB_FIX_ASPECT_RATIO +
                      CV_CALIB_ZERO_TANGENT_DIST +
                      CV_CALIB_SAME_FOCAL_LENGTH );
    printf(" done\n");
    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    vector<CvPoint3D32f> lines[2];
    points[0].resize(N);
    points[1].resize(N);
    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    lines[0].resize(N);
    lines[1].resize(N);
    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
    //Always work in undistorted space
    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
                      &_M1, &_D1, 0, &_M1 );
    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
                      &_M2, &_D2, 0, &_M2 );
    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
    double avgErr = 0;
    for( i = 0; i < N; i++ )
    {
        double err = fabs(points[0][i].x*lines[1][i].x +
                          points[0][i].y*lines[1][i].y + lines[1][i].z)
        + fabs(points[1][i].x*lines[0][i].x +
               points[1][i].y*lines[0][i].y + lines[0][i].z);
        avgErr += err;
    }
    printf( "avg err = %g\n", avgErr/(nframes*n) );
    //COMPUTE AND DISPLAY RECTIFICATION
    if( showUndistorted )
    {
        CvMat* mx1 = cvCreateMat( imageSize.height,
                                 imageSize.width, CV_32F );
        CvMat* my1 = cvCreateMat( imageSize.height,
                                 imageSize.width, CV_32F );
        CvMat* mx2 = cvCreateMat( imageSize.height,
                                 
                                 imageSize.width, CV_32F );
        CvMat* my2 = cvCreateMat( imageSize.height,
                                 imageSize.width, CV_32F );
        
        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
        CvMat _R2 = cvMat(3, 3, CV_64F, R2);
        // IF BY CALIBRATED (BOUGUET'S METHOD)
        if( useUncalibrated == 0 )
        {
            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
            cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
                            &_R, &_T,
                            &_R1, &_R2, &_P1, &_P2, &_Q,
                            0/*CV_CALIB_ZERO_DISPARITY*/ );
            isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
            //Precompute maps for cvRemap()
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
            cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
            
            if(save)
            {
                //Save parameters
                cvSave("M1.xml",&_M1);
                cvSave("D1.xml",&_D1);
                cvSave("R1.xml",&_R1);
                cvSave("P1.xml",&_P1);
                cvSave("M2.xml",&_M2);
                cvSave("D2.xml",&_D2);
                cvSave("R2.xml",&_R2);
                cvSave("P2.xml",&_P2);
                cvSave("Q.xml",&_Q);
                cvSave("mx1.xml",mx1);
                cvSave("my1.xml",my1);
                cvSave("mx2.xml",mx2);
                cvSave("my2.xml",my2);
            }
        }
        cvReleaseMat(&mx1);
        cvReleaseMat(&mx2);
        cvReleaseMat(&my1);
        cvReleaseMat(&my2);
    }
    
}      

void Camera::showDepthMap()
{
    if(!cameraLeft || !cameraRight)
        setCameras(0, 2);
    if(!mBMState){
        mBMState = cvCreateStereoBMState();
        setDefaultBMState();
    }
    
    IplImage* img1;
    IplImage* img2;
    CvMat* img1r = cvCreateMat( imageSize.height, imageSize.width, CV_8U );
    CvMat* img2r = cvCreateMat( imageSize.height, imageSize.width, CV_8U );
    CvMat* disp  = cvCreateMat( imageSize.height, imageSize.width, CV_16S);
    CvMat* vdisp = cvCreateMat( imageSize.height, imageSize.width, CV_8U );
    
    assert(mBMState != 0);
    
    cvNamedWindow( "disparity" );
    bool quit = false;
    while(!quit){
        img1= cvCloneImage(cvQueryFrame(cameraLeft));
        img2= cvCloneImage(cvQueryFrame(cameraRight));        
        if( img1 && img2 )
        {
            cvRemap( img1, img1r, mx1, my1 );
            cvRemap( img2, img2r, mx2, my2 );
        } else
            continue;
        // When the stereo camera is oriented vertically,
        // useUncalibrated==0 does not transpose the
        // image, so the epipolar lines in the rectified
        // images are vertical. Stereo correspondence
        // function does not support such a case.
        cvFindStereoCorrespondenceBM( img1r, img2r, disp,
                                     mBMState);
        cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
        cvShowImage( "disparity", vdisp );
        if (cvWaitKey(33) == 27) {
            break;
        }
    }
    cvReleaseImage( &img1 );
    cvReleaseImage( &img2 );
    cvReleaseStereoBMState(&mBMState);
    cvReleaseMat( &img1r );
    cvReleaseMat( &img2r );
    cvReleaseMat( &disp );
}

