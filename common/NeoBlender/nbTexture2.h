//
//  flTexture.h
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 29..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#ifndef __NeoBlender__flTexture2__
#define __NeoBlender__flTexture2__

#include "nbMemory.h"
#include "nbTexture.h"

namespace nbd {
    
//    enum
//    {
//        //! Enable image clamping on the UV, 0 to 1 only no texture repeat.
//        TEXTURE_CLAMP = ( 1 << 0 ),
//        
//        //! Enable automatic mipmap generation.
//        TEXTURE_MIPMAP = ( 1 << 1 ),
//        
//        //! Enable automatic 16 bits conversion.
//        TEXTURE_16_BITS = ( 1 << 2 ),
//        
//        //! Force the conversion of 32 bits textures to use use 5551 instead of 4444.
//        TEXTURE_16_BITS_5551 = ( 1 << 3 )
//    };
//    
//    
//    enum
//    {
//        //! Image filtering nearest.
//        TEXTURE_FILTER_0X = 0,
//        
//        //! Image filtering linear.
//        TEXTURE_FILTER_1X = 1,
//        
//        //! Bilinear image filtering.
//        TEXTURE_FILTER_2X = 2,
//        
//        //! Trilinear image filtering.
//        TEXTURE_FILTER_3X = 3
//    };
//    
//    
//    //! PVRTC file header data.
//    typedef struct
//    {
//        //! The size of the PVRTC header.
//        unsigned int headersize;
//        
//        //! The height of the original texture size.
//        unsigned int height;
//        
//        //! The width of the original texture size.
//        unsigned int width;
//        
//        //! The number of number of mipmap contained in the PVR image stream.
//        unsigned int n_mipmap;
//        
//        //! The flags of the PVR image.
//        unsigned int flags;
//        
//        //! Data block size.
//        unsigned int datasize;
//        
//        //! The number of bits per pixel.
//        unsigned int bpp;
//        
//        //! The size of the red bit.
//        unsigned int bitred;
//        
//        //! The size of the green bit.
//        unsigned int bitgreen;
//        
//        //! The size of the blue bit.
//        unsigned int bitblue;
//        
//        //! The size of the alpha bit.
//        unsigned int bitalpha;
//        
//        //! Unknown.
//        unsigned int tag;
//        
//        //! The number of surface held in the PVR file.
//        unsigned int n_surface;
//        
//    } PVRHEADER;
    
    class Texture2 {
        
    public:
        char			fullPath[ MAX_PATH ];
        
        //! Internal name to use for the TEXTURE.
        char			name[ MAX_CHAR ];
        
        //! The OpenGLES texture id.
        unsigned int	tid;
        
        //! The texture width.
        unsigned short	width;
        
        //! The texture height.
        unsigned short	height;
        
        //! The texture bits.
        unsigned char	byte;
        
        //! The total size of the texture in bytes.
        unsigned int	size;
        
        //! The target of the texture. (Default GL_TEXTURE_2D)
        unsigned int	target;
        
        //! The internal format of the texture.
        unsigned int	internal_format;
        
        //! The format of the texture.
        unsigned int	format;
        
        //! The type of texel to use for this texture. (Default GL_UNSIGNED_BYTE)
        unsigned int	texel_type;
        
        //! The raw texel array.
        unsigned char	*texel_array;
        
        //! The number of mipmaps (PVRTC only).
        unsigned int	n_mipmap;
        
        //! The compression type.
        unsigned int	compression;
        
    public:
        
        Texture2(char *name);
        
        virtual ~Texture2();
        
        void init(char *name);
        
        //    TEXTURE *TEXTURE_init( char *name );
        
        //    TEXTURE *TEXTURE_free( TEXTURE *texture );
        
        Texture2 *create(char *name, char *filename, unsigned char relative_path, unsigned int flags, unsigned char filter, float anisotropic_filter);
        
        void build(unsigned int  flags,
                   unsigned char filter,
                   float		  anisotropic_filter);
        
        void load(Memory *memory);
        
        void load_png(Memory *memory);
        
        void load_pvr(Memory *memory);
        
        void convert_16_bits(unsigned char use_5551);
        
        void generate_id(unsigned int flags, unsigned char filter, float anisotropic_filter);
        
        void delete_id();
        
        void free_texel_array();
        
        void draw();
        
        void scale(unsigned int width, unsigned int height);
        
        
    };
    
}


#endif /* defined(__NeoBlender__flTexture2__) */
