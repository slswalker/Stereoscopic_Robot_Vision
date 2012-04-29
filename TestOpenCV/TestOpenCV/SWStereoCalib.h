//
//  SWStereoCalib.h
//  TestOpenCV
//
//  Created by Sam Walker on 1/23/12.
//  Copyright (c) 2012 Sam Walker. All rights reserved.
//

#ifndef TestOpenCV_SWStereoCalib_h
#define TestOpenCV_SWStereoCalib_h

#include <vector>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "cxmisc.h"
#include "highgui.h"
#include "cvaux.h"
void stereoCalibrate(int nx, int ny, float squareSize, IplImage* img[PICTURES][2], int useUncalibrated,
                     CvCapture* leftCapture, CvCapture* rightCapture, IplImage* finalImage)
{
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<CvPoint2D32f> corners(nx*ny);
    CvSize imageSize = {0,0};
    bool isVerticalStereo = false;
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
    int nFrames = 0;
    int N = 0;
    int i = 0;
    int j = 0;
    int number = nx*ny;
    std::cout << "Done initializing variables.\n";
    cout << (img[0][0]) << endl;
    imageSize = cvGetSize(img[0][0]);
    std::cout << imageSize.width << " " << imageSize.height << std::endl;
    for (i = 0;; i++) {
        int count = 0;
        int result = 0;
        int result2 = 0;
        int lr = i % 2;
        if (i >= PICTURES*2) break;
        vector<CvPoint2D32f>& pts1 = points[lr];
        //vector<CvPoint2D32f>& pts2 = points[lr2];
        IplImage* timg1 = img[i/2][lr];
        timg1 = cvCreateImage(cvSize(img[i/2][lr]->width,img[i/2][lr]->height),
                              img[i/2][lr]->depth, img[i/2][lr]->nChannels );
        cvResize( img[i/2][lr], timg1, CV_INTER_CUBIC );
        
        result = cvFindChessboardCorners( timg1, cvSize(nx, ny),
                                         &corners[0], &count,
                                         CV_CALIB_CB_ADAPTIVE_THRESH |
                                         CV_CALIB_CB_NORMALIZE_IMAGE);
        
        if( timg1 != img[i/2][lr] )
            cvReleaseImage( &timg1 );
        
        if (!result)
        {
            continue;
        }
        if (lr == 1) {
            nFrames++;
        }
        if (result) 
        {
            IplImage* vimg = cvCreateImage(cvSize(img[i/2][lr]->width,img[i/2][lr]->height),
                                           img[i/2][lr]->depth, 1  );
            cvFindCornerSubPix( vimg, &corners[0], count,
                               cvSize(11, 11), cvSize(-1,-1),
                               cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                              30, 0.01) );
            N = pts1.size();
            pts1.resize(N + (number), cvPoint2D32f(0,0));
            copy( corners.begin(), corners.end(), pts1.begin() + N );
            cvReleaseImage( &img[i/2][lr] );
        }
        
        
        /*IplImage* timg2 = img[i][lr2];
        timg2 = cvCreateImage(cvSize(img[i][lr2]->width,img[i][lr2]->height),
                              img[i][lr2]->depth, img[i][lr2]->nChannels );
        cvResize( img[i][lr2], timg2, CV_INTER_CUBIC );
        
        result2 = cvFindChessboardCorners( timg2, cvSize(nx, ny),
                                          &corners[0], &count,
                                          CV_CALIB_CB_ADAPTIVE_THRESH |
                                          CV_CALIB_CB_NORMALIZE_IMAGE);
        
        
        if( timg2 != img[i][lr2] )
            cvReleaseImage( &timg2 );
        
        nFrames++;
        
        if (result2)
        {
            IplImage* vimg = cvCreateImage(cvSize(img[i][lr2]->width,img[i][lr2]->height),
                                           img[i][lr2]->depth, 1  );
            cvFindCornerSubPix( vimg, &corners[0], count,
                               cvSize(11, 11), cvSize(-1,-1),
                               cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                              30, 0.01) );
            N = pts2.size();
            pts2.resize(N + (number), cvPoint2D32f(0,0));
            copy( corners.begin(), corners.end(), pts2.begin() + N );
            cvReleaseImage( &img[i][lr2] );
        }*/
    }
    cout << nFrames << "\n";
    objectPoints.resize(nFrames*number);
    
    for( i = 0; i < ny; i++ )
        for( j = 0; j < nx; j++ )
            objectPoints[i*nx + j] = cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < nFrames; i++ )
        copy( objectPoints.begin(), objectPoints.begin() + number,
             objectPoints.begin() + i*number );
    
    npoints.resize(nFrames,number);
    N = nFrames*number;
    
    
    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
    cvSetIdentity(&_M1);
    cvSetIdentity(&_M2);
    cvZero(&_D1);
    cvZero(&_D2);
    
    //    if (CV_IS_MAT(&_objectPoints))
    //        std::cout << "Object points is valid.\n";
    //    
    //    if (CV_IS_MAT(&_imagePoints1))
    //        std::cout << "_imagePoints1 points is valid.\n";
    //    
    //    if (CV_IS_MAT(&_imagePoints2))
    //        std::cout << "_imagePoints2 points is valid.\n";
    //    
    //    if (CV_IS_MAT(&_npoints))
    //        std::cout << "_npoints points is valid.\n";
    //    
    //    if (CV_IS_MAT(&_T))
    //        std::cout << "_T points is valid.\n";
    //    
    //    if (CV_IS_MAT(&_R))
    //        std::cout << "_R points is valid.\n";
    printf("Running stereo calibration ...");
    cvStereoCalibrate( &_objectPoints, &_imagePoints1,
                      &_imagePoints2, &_npoints,
                      &_M1, &_D1, &_M2, &_D2,
                      imageSize, &_R, &_T, &_E, &_F,
                      cvTermCriteria(CV_TERMCRIT_ITER+
                                     CV_TERMCRIT_EPS, 100, 1e-5),
                      CV_CALIB_FIX_ASPECT_RATIO +
                      CV_CALIB_ZERO_TANGENT_DIST +
                      CV_CALIB_SAME_FOCAL_LENGTH );
    printf(" done\n\n");
    ////////////////////////////
    //Check quality
    vector<CvPoint3D32f> lines[2];
    points[0].resize(N);
    points[1].resize(N);
    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    lines[0].resize(N);
    lines[1].resize(N);
    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
    
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
    ///////////////////////////
    
    //IplImage*
    //cvCreateMat(<#int rows#>, <#int cols#>, <#int type#>)
    
    
    //m1type == CV_16SC2 || m1type == CV_32FC1 || m1type == CV_32FC2
    
    CvMat* mx1 = cvCreateMat( imageSize.height,
                             imageSize.width, CV_32F );
    CvMat* my1 = cvCreateMat( imageSize.height,
                             imageSize.width, CV_32F );
    CvMat* mx2 = cvCreateMat( imageSize.height,
                             imageSize.width, CV_32F );
    
    CvMat* my2 = cvCreateMat( imageSize.height,
                             imageSize.width, CV_32F );
    CvMat* img1r = cvCreateMat( imageSize.height,
                               imageSize.width, CV_8U );
    CvMat* img2r = cvCreateMat( imageSize.height,
                               imageSize.width, CV_8U );
    CvMat* disp = cvCreateMat( imageSize.height,
                              imageSize.width, CV_16S );
    CvMat* vdisp = cvCreateMat( imageSize.height,
                               imageSize.width, CV_8U );
    CvMat* pair;
    double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
    CvMat _R1 = cvMat(3, 3, CV_64F, R1);
    CvMat _R2 = cvMat(3, 3, CV_64F, R2);
    if( useUncalibrated == 0 )
    {
        CvMat _P1 = cvMat(3, 4, CV_64F, P1);
        CvMat _P2 = cvMat(3, 4, CV_64F, P2);
        cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
                        &_R, &_T,
                        &_R1, &_R2, &_P1, &_P2, &_Q,
                        0/*CV_CALIB_ZERO_DISPARITY*/ );
        //&_M1,&_D1,&_R1,&_P1,mx1,my1
        
        isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
        //Precompute maps for cvRemap()
        cout << "Distorting Rectify Map 1.\n";
        cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
        //SWInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
        cout << "Distorting Rectify Map 2.\n";
        //SWInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
        cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
        
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
    else if( useUncalibrated == 1 || useUncalibrated == 2 )
        // use intrinsic parameters of each camera, but
        // compute the rectification transformation directly
        // from the fundamental matrix
    {
        double H1[3][3], H2[3][3], iM[3][3];
        CvMat _H1 = cvMat(3, 3, CV_64F, H1);
        CvMat _H2 = cvMat(3, 3, CV_64F, H2);
        CvMat _iM = cvMat(3, 3, CV_64F, iM);
        //Just to show you could have independently used F
        if( useUncalibrated == 2 )
            cvFindFundamentalMat( &_imagePoints1,
                                 &_imagePoints2, &_F);
        cvStereoRectifyUncalibrated( &_imagePoints1,
                                    &_imagePoints2, &_F,
                                    imageSize,
                                    &_H1, &_H2, 3);
        cvInvert(&_M1, &_iM);
        cvMatMul(&_H1, &_M1, &_R1);
        cvMatMul(&_iM, &_R1, &_R1);
        cvInvert(&_M2, &_iM);
        cvMatMul(&_H2, &_M2, &_R2);
        cvMatMul(&_iM, &_R2, &_R2);
        //Precompute map for cvRemap()
        SWInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_M1,mx1,my1);
        
        SWInitUndistortRectifyMap(&_M2,&_D1,&_R2,&_M2,mx2,my2);
    }
    
    //showBMState(imageSize, pair, img1r, img2r, mx1, my1, mx2, my2, disp, vdisp, isVerticalStereo, nFrames, img, useUncalibrated);
    showBMState(imageSize, pair, img1r, img2r, mx1, my1, mx2, my2, disp, vdisp, isVerticalStereo, nFrames, leftCapture, rightCapture, useUncalibrated);
    cvReleaseMat( &mx1 );
    cvReleaseMat( &my1 );
    cvReleaseMat( &mx2 );
    cvReleaseMat( &my2 );
    cvReleaseMat( &img1r );
    cvReleaseMat( &img2r );
    cvReleaseMat( &disp );
}

#endif
