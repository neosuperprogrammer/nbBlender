//
//  nbLog.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 2. 8..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#ifndef NeoBlender_nbLog_h
#define NeoBlender_nbLog_h

#ifdef DEBUG
#   define flLog(fmt, ...)				NSLog((@"%s(%d) * " fmt " *"), __PRETTY_FUNCTION__, __LINE__, ##__VA_ARGS__)


//#   define glLog(fmt, ...)				printf(("%s(%d) * " fmt "\n"),  __PRETTY_FUNCTION__, __LINE__, ##__VA_ARGS__)
#   define glLog(fmt, ...)				printf((fmt "\n"), ##__VA_ARGS__)


//#   define glLogVerbose(fmt, ...)		printf(("%s(%d) * " fmt "\n"),  __PRETTY_FUNCTION__, __LINE__, ##__VA_ARGS__)
//#   define glLogVerbose(fmt, ...)		printf((fmt "\n"), ##__VA_ARGS__)
#   define glLogVerbose(fmt, ...)
#else
#   define flLog(fmt, ...)
#   define glLog(fmt, ...)
#   define glLogVerbose(fmt, ...)
#endif

#endif
