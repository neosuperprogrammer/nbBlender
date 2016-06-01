//
//  main.m
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 8..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "AppDelegate.h"

int main(int argc, char * argv[]) {
    setenv( "FILESYSTEM", argv[ 0 ], 1 );

    @autoreleasepool {
        return UIApplicationMain(argc, argv, nil, NSStringFromClass([AppDelegate class]));
    }
}
