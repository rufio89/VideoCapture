//
//  OpenCVWrapper.m
//  VideoCapture
//
//  Created by Ryan Krienitz on 5/10/18.
//  Copyright © 2018 com. All rights reserved.
//

#import "OpenCVWrapper.h"

// import necessary headers
#import <opencv2/core.hpp>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/imgproc/imgproc.hpp>
#import <opencv2/highgui.hpp>
#import <opencv2/core.hpp>
#import <opencv2/videoio.hpp>
#import <opencv2/videoio/cap_ios.h>

using namespace cv;

@implementation OpenCVWrapper

- (UIImage *) findHand: (UIImage *) image {
    Mat inputImage; UIImageToMat(image, inputImage);
    
    
    Mat mat;
    return MatToUIImage(mat)
}

- (UIImage *) makeGray: (UIImage *) image {
    // Convert UIImage to cv::Mat
    Mat inputImage; UIImageToMat(image, inputImage);
    // If input image has only one channel, then return image.
    if (inputImage.channels() == 1) return image;
    // Convert the default OpenCV's BGR format to GrayScale.
    Mat gray; cvtColor(inputImage, gray, CV_BGR2GRAY);
    // Convert the GrayScale OpenCV Mat to UIImage and return it.
    return MatToUIImage(gray);
}




@end
