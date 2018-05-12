//
//  OpenCVWrapper.h
//  VideoCapture
//
//  Created by Ryan Krienitz on 5/10/18.
//  Copyright © 2018 com. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

@interface OpenCVWrapper : NSObject

- (UIImage *) makeGray: (UIImage *) image;
- (UIImage *) findHand: (UIImage *) image;
- (UIImage *) HSVDetect: (UIImage *) image;
- (UIImage *) Test: (UIImage *) image;



@end
