//
//  EAGLView.m
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 28..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#import "EAGLView.h"
//#import "blenderApp.h"
#import "nbManager.h"
#import "nbLog.h"

@implementation EAGLView {
    EAGLContext *context;
    
    // The pixel dimensions of the CAEAGLLayer.
    GLint framebufferWidth;
    GLint framebufferHeight;
    
    // The OpenGL ES names for the framebuffer and renderbuffer used to render to this view.
    GLuint defaultFramebuffer, colorRenderbuffer, depthRenderbuffer;
    
//    BlenderApp *_app;
    nbManager *_mgr;
}

// You must implement this method
+ (Class)layerClass
{
    return [CAEAGLLayer class];
}

//The EAGL view is stored in the nib file. When it's unarchived it's sent -initWithCoder:.
- (id)initWithCoder:(NSCoder*)coder
{
    self = [super initWithCoder:coder];
    if (self)
    {
        CAEAGLLayer *eaglLayer = (CAEAGLLayer *)self.layer;
        
        // kEAGLColorFormatRGBA8
        // kEAGLColorFormatRGB565
        eaglLayer.opaque = TRUE;
        eaglLayer.drawableProperties = [ NSDictionary dictionaryWithObjectsAndKeys:
                                        [ NSNumber numberWithBool:FALSE ],
                                        kEAGLDrawablePropertyRetainedBacking,
                                        kEAGLColorFormatRGB565,
                                        kEAGLDrawablePropertyColorFormat,
                                        nil ];
        
        // Enable multitouch
        [ self setMultipleTouchEnabled:YES ];
        
//        _app = new BlenderApp();
        _mgr = new nbManager();
    }
    
    return self;
}

- (void)dealloc
{
    [self deleteFramebuffer];
//    delete _app;
    delete _mgr;
}

- (EAGLContext *)context
{
    return context;
}

- (void)setContext:(EAGLContext *)newContext
{
    if (context != newContext)
    {
        [self deleteFramebuffer];
        
        context = newContext;
        
        [EAGLContext setCurrentContext:nil];
    }
}

- (void)createFramebuffer
{
    if (context && !defaultFramebuffer )
    {
        [EAGLContext setCurrentContext:context];
        
        // Create default framebuffer object.
        glLog("glGenFramebuffers");
        glGenFramebuffers(1, &defaultFramebuffer);
        glLog("glBindFramebuffer");
        glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebuffer);
        
        // Create color render buffer and allocate backing store.
        glLog("glGenRenderbuffers : colorRenderbuffer");
        glGenRenderbuffers(1, &colorRenderbuffer);
        glLog("glBindRenderbuffer : colorRenderbuffer");
        glBindRenderbuffer(GL_RENDERBUFFER, colorRenderbuffer);
        
        glLog("renderbufferStorage : GL_RENDERBUFFER");
        [context renderbufferStorage:GL_RENDERBUFFER fromDrawable:(CAEAGLLayer *)self.layer];
        
        glLog("glGetRenderbufferParameteriv : GL_RENDERBUFFER_WIDTH");
        glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &framebufferWidth);
        glLog("glGetRenderbufferParameteriv : GL_RENDERBUFFER_HEIGHT");
        glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &framebufferHeight);
        glLog("glFramebufferRenderbuffer : colorRenderbuffer");
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorRenderbuffer);
        
        
        glLog("glGenRenderbuffers : depthRenderbuffer");
        glGenRenderbuffers(1, &depthRenderbuffer);
        glLog("glBindRenderbuffer : depthRenderbuffer");
        glBindRenderbuffer(GL_RENDERBUFFER, depthRenderbuffer);
        
        // GL_DEPTH24_STENCIL8_OES: Set a depth buffer and a stencil buffer.
        // GL_DEPTH_COMPONENT16: Set a 16bits depth buffer.
        // GL_DEPTH_COMPONENT24_OES: Set a 24bits depth buffer.
        glLog("glRenderbufferStorage : GL_DEPTH_COMPONENT16");
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, framebufferWidth, framebufferHeight);
        glLog("glFramebufferRenderbuffer : depthRenderbuffer");
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRenderbuffer);
        //glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depthRenderbuffer);
        
        glLog("glCheckFramebufferStatus : GL_FRAMEBUFFER");
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            NSLog(@"Failed to make complete framebuffer object %x", glCheckFramebufferStatus(GL_FRAMEBUFFER));
    }
}

- (void)deleteFramebuffer
{
    if (context)
    {
        [EAGLContext setCurrentContext:context];
        
        if (defaultFramebuffer)
        {
            glLog("glDeleteFramebuffers : defaultFramebuffer");
            glDeleteFramebuffers(1, &defaultFramebuffer);
            defaultFramebuffer = 0;
        }
        
        if (colorRenderbuffer)
        {
            glLog("glDeleteRenderbuffers : colorRenderbuffer");
            glDeleteRenderbuffers(1, &colorRenderbuffer);
            colorRenderbuffer = 0;
        }
        
        
        if( depthRenderbuffer )
        {
            glLog("glDeleteRenderbuffers : depthRenderbuffer");
            glDeleteRenderbuffers(1, &depthRenderbuffer);
            depthRenderbuffer = 0;
        }
    }
}

- (void)setFramebuffer
{
    if (context)
    {
        glLogVerbose("EAGLView::setFrameBuffer");
        [EAGLContext setCurrentContext:context];
        
        if (!defaultFramebuffer)
            [self createFramebuffer];
        
        glLogVerbose("glBindFramebuffer : defaultFramebuffer");
        glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebuffer);
    }
}

- (void)initApp
{
    int width, height;
    // Initialize the template App.
    glLog("glGetRenderbufferParameteriv : GL_RENDERBUFFER_WIDTH");
    glGetRenderbufferParameteriv( GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH , &width  );
    glLog("glGetRenderbufferParameteriv : GL_RENDERBUFFER_HEIGHT");
    glGetRenderbufferParameteriv( GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &height );
    
//    _app->init( width, height );
    _mgr->start(width, height);
}

- (void)draw
{
//    _app->draw();
    _mgr->draw();
}

- (BOOL)presentFramebuffer
{
    BOOL success = FALSE;
    
    if (context)
    {
        glLogVerbose("EAGLView::presentFramebuffer");

        [EAGLContext setCurrentContext:context];
        
        glLogVerbose("glBindRenderbuffer : colorRenderbuffer");
        glBindRenderbuffer(GL_RENDERBUFFER, colorRenderbuffer);
        
        glLogVerbose("context presentRenderbuffer : GL_RENDERBUFFER");
        success = [context presentRenderbuffer:GL_RENDERBUFFER];
    }
    
    return success;
}

- (void)layoutSubviews
{
    // The framebuffer will be re-created at the beginning of the next setFramebuffer method call.
    [self deleteFramebuffer];
}


- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch *touch;
    CGPoint pos;
    
    for( touch in touches )
    {
        pos = [ touch locationInView:self ];
        _mgr->toucheBegan(pos.x,
                          pos.y,
                          (unsigned int)[[touches anyObject] tapCount]);
        
//        if( blenderApp.ToucheBegan )
//        {
//            _app->toucheBegan( pos.x,
//                                    pos.y,
//                                    (unsigned int)[ [ touches anyObject ] tapCount ] );
//        }
    }
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch *touch;
    CGPoint pos;
    
    for( touch in touches )
    {
        pos = [ touch locationInView:self ];
        _mgr->toucheMoved(pos.x,
                          pos.y,
                          (unsigned int)[[touches anyObject] tapCount]);
//        if( blenderApp.ToucheMoved )
//        {
//            _app->toucheMoved( pos.x,
//                                    pos.y,
//                                    (unsigned int)[ [ touches anyObject ] tapCount ] );
//        }
    }
}

- (void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch *touch;
    CGPoint pos;
    
    for( touch in touches )
    {
        pos = [ touch locationInView:self ];
        
//        if( blenderApp.ToucheEnded )
//        {
//            _app->toucheEnded( pos.x,
//                                    pos.y,
//                                    (unsigned int)[ [ touches anyObject ] tapCount ] );
//        }
    }
}

@end
