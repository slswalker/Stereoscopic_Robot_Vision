//
//  openCVHelper.h
//  TestOpenCV
//
//  Created by Test on 1/18/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef TestOpenCV_openCVHelper_h
#define TestOpenCV_openCVHelper_h
void initUndistortRectifyMap( const Mat& _cameraMatrix, const Mat& _distCoeffs,
                     const Mat& _R, const Mat& _newCameraMatrix,
                                                         Size size, int m1type, Mat& map1, Mat& map2 )
	{
    68	    if( m1type <= 0 )
        69	        m1type = CV_16SC2;
        70	    CV_Assert( m1type == CV_16SC2 || m1type == CV_32FC1 || m1type == CV_32FC2 );
        71	    map1.create( size, m1type );
        72	    if( m1type != CV_32FC2 )
            73	        map2.create( size, m1type == CV_16SC2 ? CV_16UC1 : CV_32FC1 );
            74	    else
                75	        map2.release();
                76	
                77	    Mat_<double> R = Mat_<double>::eye(3, 3), distCoeffs;
                78	    Mat_<double> A = Mat_<double>(_cameraMatrix), Ar;
                79	
                80	    if( _newCameraMatrix.data )
                    81	        Ar = Mat_<double>(_newCameraMatrix);
                    82	    else
                        83	        Ar = getDefaultNewCameraMatrix( A, size, true );
                        84	
                        85	    if( _R.data )
                            86	        R = Mat_<double>(_R);
                            87	
                            88	    if( _distCoeffs.data )
                                89	        distCoeffs = Mat_<double>(_distCoeffs);
                                90	    else
                                    91	    {
                                        92	        distCoeffs.create(5, 1);
                                        93	        distCoeffs = 0.;
                                        94	    }
    95	
    96	    CV_Assert( A.size() == Size(3,3) && A.size() == R.size() );
    97	    CV_Assert( Ar.size() == Size(3,3) || Ar.size() == Size(4, 3));
    98	    Mat_<double> iR = (Ar.colRange(0,3)*R).inv(DECOMP_LU);
    99	    const double* ir = &iR(0,0);
    100	
    101	    double u0 = A(0, 2),  v0 = A(1, 2);
    102	    double fx = A(0, 0),  fy = A(1, 1);
    103	
    104	    CV_Assert( distCoeffs.size() == Size(1, 4) || distCoeffs.size() == Size(1, 5) ||
                      105	               distCoeffs.size() == Size(4, 1) || distCoeffs.size() == Size(5, 1));
    106	
    107	    if( distCoeffs.rows != 1 && !distCoeffs.isContinuous() )
        108	        distCoeffs = distCoeffs.t();
        109	
        110	    double k1 = ((double*)distCoeffs.data)[0];
        111	    double k2 = ((double*)distCoeffs.data)[1];
        112	    double p1 = ((double*)distCoeffs.data)[2];
        113	    double p2 = ((double*)distCoeffs.data)[3];
        114	    double k3 = distCoeffs.cols + distCoeffs.rows - 1 == 5 ? ((double*)distCoeffs.data)[4] : 0.;
        115	
        116	    for( int i = 0; i < size.height; i++ )
            117	    {
                118	        float* m1f = (float*)(map1.data + map1.step*i);
                119	        float* m2f = (float*)(map2.data + map2.step*i);
                120	        short* m1 = (short*)m1f;
                121	        ushort* m2 = (ushort*)m2f;
                122	        double _x = i*ir[1] + ir[2], _y = i*ir[4] + ir[5], _w = i*ir[7] + ir[8];
                123	
                124	        for( int j = 0; j < size.width; j++, _x += ir[0], _y += ir[3], _w += ir[6] )
                    125	        {
                        126	            double w = 1./_w, x = _x*w, y = _y*w;
                        127	            double x2 = x*x, y2 = y*y;
                        128	            double r2 = x2 + y2, _2xy = 2*x*y;
                        129	            double kr = 1 + ((k3*r2 + k2)*r2 + k1)*r2;
                        130	            double u = fx*(x*kr + p1*_2xy + p2*(r2 + 2*x2)) + u0;
                        131	            double v = fy*(y*kr + p1*(r2 + 2*y2) + p2*_2xy) + v0;
                        132	            if( m1type == CV_16SC2 )
                            133	            {
                                134	                int iu = saturate_cast<int>(u*INTER_TAB_SIZE);
                                135	                int iv = saturate_cast<int>(v*INTER_TAB_SIZE);
                                136	                m1[j*2] = (short)(iu >> INTER_BITS);
                                137	                m1[j*2+1] = (short)(iv >> INTER_BITS);
                                138	                m2[j] = (ushort)((iv & (INTER_TAB_SIZE-1))*INTER_TAB_SIZE + (iu & (INTER_TAB_SIZE-1)));
                                139	            }
                        140	            else if( m1type == CV_32FC1 )
                            141	            {
                                142	                m1f[j] = (float)u;
                                143	                m2f[j] = (float)v;
                                144	            }
                        145	            else
                            146	            {
                                147	                m1f[j*2] = (float)u;
                                148	                m1f[j*2+1] = (float)v;
                                149	            }
                        150	        }
                151	    }
    152	}


#endif
