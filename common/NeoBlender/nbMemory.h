//
//  flMemory.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 29..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#ifndef __NeoBlender__flMemory__
#define __NeoBlender__flMemory__

#include "nbTypes.h"

namespace nbd {
    
class Memory {
    
public:
    //! The filename used by the MEMORY structure.
    char			filename[ MAX_PATH ];
    
    //! The file size in bytes.
    unsigned int	size;
    
    //! The position of the cursor within the memory buffer.
    unsigned int	position;
    
    //! The memory buffer.
    unsigned char	*buffer;
   
public:
    
    virtual ~Memory();
    
    void close();
    
    static Memory *open(char *filename, unsigned char relative_path);
    
    unsigned int read(void *dst, unsigned int size );
    
    void insert(char *str, unsigned int position );
    
};
    
}

#endif /* defined(__NeoBlender__flMemory__) */
