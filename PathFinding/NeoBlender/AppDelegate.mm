//
//  AppDelegate.m
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 8..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#import "AppDelegate.h"
#import "ViewController.h"

@interface AppDelegate ()

@end

@implementation AppDelegate {
    ViewController *_viewController;
}


- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions {
    
    [[UIApplication sharedApplication] setIdleTimerDisabled:YES];

    
//    ViewController *viewController = [[UIStoryboard storyboardWithName:@"Main" bundle:nil] instantiateInitialViewController];
//    self.window.rootViewController =
    
    _viewController = (ViewController *)self.window.rootViewController;
    
    // Override point for customization after application launch.
//    CGRect screenBounds = [[UIScreen mainScreen] bounds];
//    self.glView = [[OpenGLView alloc] initWithFrame:screenBounds];
//    [self.window addSubview:_glView];
    
    return YES;
}

- (void)applicationWillResignActive:(UIApplication *)application {
    [_viewController stopAnimation];
}

- (void)applicationDidEnterBackground:(UIApplication *)application {
    // Use this method to release shared resources, save user data, invalidate timers, and store enough application state information to restore your application to its current state in case it is terminated later.
    // If your application supports background execution, this method is called instead of applicationWillTerminate: when the user quits.
}

- (void)applicationWillEnterForeground:(UIApplication *)application {
    // Called as part of the transition from the background to the inactive state; here you can undo many of the changes made on entering the background.
}

- (void)applicationDidBecomeActive:(UIApplication *)application {
   [_viewController startAnimation];
}

- (void)applicationWillTerminate:(UIApplication *)application {
    [_viewController stopAnimation];
}

@end
