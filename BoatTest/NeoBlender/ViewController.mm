//
//  ViewController.m
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 8..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#import "ViewController.h"
#import "OpenGLView.h"
#import "EAGLView.h"

@interface ViewController ()

@property (nonatomic, strong) EAGLContext *context;
@property (nonatomic) CADisplayLink *displayLink;

@property (strong, nonatomic) IBOutlet OpenGLView *openGLView;
@property (strong, nonatomic) IBOutlet EAGLView *eaglView;

@end

@implementation ViewController {
    BOOL animating;
    NSInteger animationFrameInterval;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    EAGLContext *aContext = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    
    if (!aContext)
        NSLog(@"Failed to create ES context");
    else if (![EAGLContext setCurrentContext:aContext])
        NSLog(@"Failed to set ES context current");
    
    self.context = aContext;
    
    [self.eaglView setContext:self.context];
    [self.eaglView setFramebuffer];
    
    [self.eaglView initApp];
    
    animating = FALSE;
    animationFrameInterval = 1;
    self.displayLink = nil;
}

- (BOOL)prefersStatusBarHidden
{
    return YES;
}

- (void)dealloc
{
    // Tear down context.
    if ([EAGLContext currentContext] == self.context)
        [EAGLContext setCurrentContext:nil];
}

- (void)viewWillAppear:(BOOL)animated
{
    [self startAnimation];
    
    [super viewWillAppear:animated];
}

- (void)viewWillDisappear:(BOOL)animated
{
    [self stopAnimation];
    
    [super viewWillDisappear:animated];
}

- (void)viewDidUnload
{
    [super viewDidUnload];
    
    // Tear down context.
    if ([EAGLContext currentContext] == self.context)
        [EAGLContext setCurrentContext:nil];
    self.context = nil;
}

- (NSInteger)animationFrameInterval
{
    return animationFrameInterval;
}

- (void)setAnimationFrameInterval:(NSInteger)frameInterval
{
    /*
     Frame interval defines how many display frames must pass between each time the display link fires.
     The display link will only fire 30 times a second when the frame internal is two on a display that refreshes 60 times a second. The default frame interval setting of one will fire 60 times a second when the display refreshes at 60 times a second. A frame interval setting of less than one results in undefined behavior.
     */
    if (frameInterval >= 1)
    {
        animationFrameInterval = frameInterval;
        
        if (animating)
        {
            [self stopAnimation];
            [self startAnimation];
        }
    }
}

- (void)startAnimation
{
    if (!animating)
    {
        CADisplayLink *aDisplayLink = [[UIScreen mainScreen] displayLinkWithTarget:self selector:@selector(drawFrame)];
        [aDisplayLink setFrameInterval:animationFrameInterval];
        [aDisplayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
        self.displayLink = aDisplayLink;
        
        animating = TRUE;
    }
}

- (void)stopAnimation
{
    if (animating)
    {
        [self.displayLink invalidate];
        self.displayLink = nil;
        animating = FALSE;
    }
}


- (void)drawFrame
{
    [self.eaglView setFramebuffer];
    
    [self.eaglView draw];
    
    [self.eaglView presentFramebuffer];
}
@end
