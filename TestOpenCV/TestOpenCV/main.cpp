//
//  main.h
//  TestOpenCV
//
//  Created by Sam Walker on 1/23/12.
//  Copyright (c) 2012 Sam Walker. All rights reserved.
//

#include "Camera.h"
#include <iostream>


int main(void)
{
    Camera camera;
    camera.takePictures();
    camera.stereoCalibPictures(0, 1);
    camera.showDepthMap();
    return 0;
}
