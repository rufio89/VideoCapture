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
    float innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1)
    {
        
        float dist1 = std::sqrt(  (px1-cx1)*(px1-cx1) + (py1-cy1)*(py1-cy1) );
        float dist2 = std::sqrt(  (px2-cx1)*(px2-cx1) + (py2-cy1)*(py2-cy1) );
        
        float Ax, Ay;
        float Bx, By;
        float Cx, Cy;
        
        //find closest point to C
        //printf("dist = %lf %lf\n", dist1, dist2);
        
        Cx = cx1;
        Cy = cy1;
        if(dist1 < dist2)
        {
            Bx = px1;
            By = py1;
            Ax = px2;
            Ay = py2;
            
            
        }else{
            Bx = px2;
            By = py2;
            Ax = px1;
            Ay = py1;
        }
        
        
        float Q1 = Cx - Ax;
        float Q2 = Cy - Ay;
        float P1 = Bx - Ax;
        float P2 = By - Ay;
        
        
        float A = std::acos( (P1*Q1 + P2*Q2) / ( std::sqrt(P1*P1+P2*P2) * std::sqrt(Q1*Q1+Q2*Q2) ) );
        
        A = A*180/CV_PI;
        
        return A;
    }

- (UIImage *) findHand: (UIImage *) image {
    Mat frame, src_gray, dst;
    int kernel_size = 3;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    UIImageToMat(image, frame);
    /// Remove noise by blurring with a Gaussian filter
    Mat g; cv::GaussianBlur(frame, g, cv::Size(3,3), BORDER_DEFAULT);
    
    /// Convert the image to grayscale
    cvtColor( frame, src_gray, CV_BGR2GRAY );
    
    /// Apply Laplace function
    Mat abs_dst;
    
    Laplacian( src_gray, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( dst, abs_dst );
    Mat canny; cv::Canny(abs_dst, canny, 20, 350);
    // extract contours of the canny image:
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchyH;
    cv::findContours(canny,contours, hierarchyH, CV_RETR_TREE , CV_CHAIN_APPROX_SIMPLE);
    // draw the contours to a copy of the input image:
    int largestContour = 0;
    for( int i = 0; i< contours.size(); i++ )
    {
        if (cv::contourArea(contours[i]) > cv::contourArea(contours[largestContour]))
            largestContour = i;
        
    }
    cv::drawContours( frame, contours, largestContour, cv::Scalar(0, 55,55,255), 1, 8, hierarchyH, 0);
    if (!contours.empty())
    {
        std::vector<std::vector<cv::Point> > hull(1);
        cv::convexHull(cv::Mat(contours[largestContour]), hull[0], false);
        cv::drawContours(frame, hull, 0, cv::Scalar(0, 0, 255, 0), 3);
    }
    if (!contours.empty())
    {
        std::vector<std::vector<cv::Point> > hull(1);
        cv::convexHull(cv::Mat(contours[largestContour]), hull[0], false);
        cv::drawContours(frame, hull, 0, cv::Scalar(0, 0, 255, 0), 3);
        if (hull[0].size() > 2)
        {
            std::vector<int> hullIndexes;
            cv::convexHull(cv::Mat(contours[largestContour]), hullIndexes, true);
            std::vector<cv::Vec4i> convexityDefects;
            cv::convexityDefects(cv::Mat(contours[largestContour]), hullIndexes, convexityDefects);
            cv::Rect boundingBox = cv::boundingRect(hull[0]);
            cv::rectangle(frame, boundingBox, cv::Scalar(0, 255, 0, 0));
            cv::Point center = cv::Point(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
            std::vector<cv::Point> validPoints;
            for (size_t i = 0; i < convexityDefects.size(); i++)
            {
                cv::Point p1 = contours[largestContour][convexityDefects[i][0]];
                cv::Point p2 = contours[largestContour][convexityDefects[i][1]];
                cv::Point p3 = contours[largestContour][convexityDefects[i][2]];
                double angle = std::atan2(center.y - p1.y, center.x - p1.x) * 180 / CV_PI;
                double inAngle = innerAngle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
                double length = std::sqrt(std::pow(p1.x - p3.x, 2) + std::pow(p1.y - p3.y, 2));
                if (angle > -30 && angle < 160 && std::abs(inAngle) > 20 && std::abs(inAngle) < 120 && length > 0.1 * boundingBox.height)
                {
                    validPoints.push_back(p1);
                }
            }
            for (size_t i = 0; i < validPoints.size(); i++)
            {
                cv::circle(frame, validPoints[i], 9, cv::Scalar(0, 0, 255, 0), 2);
            }
        }
    }
    
    return MatToUIImage(frame);
}

- (UIImage *) Test: (UIImage *) image {
    Mat frame;
    UIImageToMat(image, frame);
    int minH = 10, maxH = 300, minS = 10, maxS = 300, minV = 75, maxV = 130;
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, CV_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), hsv);
    // Pre processing
    int blurSize = 5;
    int elementSize = 5;
    medianBlur(hsv, hsv, blurSize);
    Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * elementSize + 1, 2 * elementSize + 1), cv::Point(elementSize, elementSize));
    dilate(hsv, hsv, element);
    // Contour detection
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(hsv, contours, hierarchy, CV_RETR_TREE , CV_CHAIN_APPROX_SIMPLE);

    cv::Mat output = frame.clone();
    int largestContour = 0;
    for (int i = 1; i < contours.size(); i++)
    {
        if (cv::contourArea(contours[i]) > cv::contourArea(contours[largestContour]))
            largestContour = i;
    }
    cv::drawContours(frame, contours, largestContour, cv::Scalar(0, 0, 0, 255), 1);
    // Convex hull
    if (!contours.empty())
    {
        std::vector<std::vector<cv::Point> > hull(1);
        cv::convexHull(cv::Mat(contours[largestContour]), hull[0], false);
        cv::drawContours(frame, hull, 0, cv::Scalar(0, 0, 255, 0), 3);
    }
    if (!contours.empty())
    {
        std::vector<std::vector<cv::Point> > hull(1);
        cv::convexHull(cv::Mat(contours[largestContour]), hull[0], false);
        cv::drawContours(frame, hull, 0, cv::Scalar(0, 0, 255, 0), 3);
        if (hull[0].size() > 2)
        {
            std::vector<int> hullIndexes;
            cv::convexHull(cv::Mat(contours[largestContour]), hullIndexes, true);
            std::vector<cv::Vec4i> convexityDefects;
            cv::convexityDefects(cv::Mat(contours[largestContour]), hullIndexes, convexityDefects);
            cv::Rect boundingBox = cv::boundingRect(hull[0]);
            cv::rectangle(frame, boundingBox, cv::Scalar(0, 255, 0, 0));
            cv::Point center = cv::Point(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
            std::vector<cv::Point> validPoints;
            for (size_t i = 0; i < convexityDefects.size(); i++)
            {
                cv::Point p1 = contours[largestContour][convexityDefects[i][0]];
                cv::Point p2 = contours[largestContour][convexityDefects[i][1]];
                cv::Point p3 = contours[largestContour][convexityDefects[i][2]];
                double angle = std::atan2(center.y - p1.y, center.x - p1.x) * 180 / CV_PI;
                double inAngle = innerAngle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
                double length = std::sqrt(std::pow(p1.x - p3.x, 2) + std::pow(p1.y - p3.y, 2));
                if (angle > -30 && angle < 160 && std::abs(inAngle) > 20 && std::abs(inAngle) < 120 && length > 0.1 * boundingBox.height)
                {
                    validPoints.push_back(p1);
                }
            }
            for (size_t i = 0; i < validPoints.size(); i++)
            {
                cv::circle(frame, validPoints[i], 9, cv::Scalar(0, 0, 255, 0), 2);
            }
        }
    }
    
    return MatToUIImage(frame);

}

- (UIImage *) HSVDetect: (UIImage *) image {
    Mat src;
    cv::Mat hsv;
    UIImageToMat(image, src);
    cv::cvtColor(src,hsv,CV_BGR2HSV);
    
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    
    cv::Mat H = channels[0];
    cv::Mat S = channels[1];
    cv::Mat V = channels[2];
    
    cv::Mat shiftedH = S.clone();
    int shift = 25; // in openCV hue values go from 0 to 180 (so have to be doubled to get to 0 .. 360) because of byte range from 0 to 255
    for(int j=0; j<shiftedH.rows; ++j)
        for(int i=0; i<shiftedH.cols; ++i)
        {
            shiftedH.at<unsigned char>(j,i) = (shiftedH.at<unsigned char>(j,i) + shift)%180;
        }
    cv::Mat cannyH;
    cv::Canny(shiftedH, cannyH, 30, 150);


    // extract contours of the canny image:
    std::vector<std::vector<cv::Point> > contoursH;
    std::vector<cv::Vec4i> hierarchyH;
    cv::findContours(cannyH,contoursH, hierarchyH, CV_RETR_TREE , CV_CHAIN_APPROX_SIMPLE);

    cv::Mat outputH = src.clone();
    for( int i = 0; i< contoursH.size(); i++ )
    {
        if(cv::contourArea(contoursH[i]) < 20) continue; // ignore contours that are too small to be a patty
        if(hierarchyH[i][3] < 0) continue;  // ignore "outer" contours

        cv::drawContours( outputH, contoursH, i, cv::Scalar(0,0,255), 2, 8, hierarchyH, 0);
    }
    
    return MatToUIImage(outputH);
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
