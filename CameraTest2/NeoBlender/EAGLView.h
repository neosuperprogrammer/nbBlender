//
//  EAGLView.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 28..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <OpenGLES/ES2/gl.h>
#import <OpenGLES/ES2/glext.h>

@interface EAGLView : UIView

- (void)setContext:(EAGLContext *)newContext;
- (void)setFramebuffer;
- (void)initApp;
- (void)draw;
- (BOOL)presentFramebuffer;

@end
