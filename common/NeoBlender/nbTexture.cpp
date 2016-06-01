//
//  flTexture.cpp
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 29..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#include "nbTexture.h"
#include "nbHeader.h"

namespace nbd {
    
    Texture::Texture(char *name)
    {
        init(name);
    }
    
    void Texture::init(char *name)
    {
        target = GL_TEXTURE_2D;
        strcpy(this->name, name);
        
        tid = 0;
        width = 0;
        height = 0;
        byte = 0;
        size = 0;
        internal_format = 0;
        format = 0;
        texel_type = 0;
        texel_array = NULL;
        n_mipmap = 0;
        compression = 0;
    }
    
    
    Texture::~Texture()
    {
        free_texel_array();
        delete_id();
    }
    
    Texture *Texture::create(char *name, char *filename, unsigned char relative_path, unsigned int flags, unsigned char filter, float anisotropic_filter)
    {
        Texture *texture = new Texture(name);
        
        Memory *m = Memory::open(filename, relative_path);
        if (m) {
            load(m);
            scale(nextPot(width), nextPot(height));
            generate_id(flags, filter, anisotropic_filter);
            free_texel_array();
            delete m;
        }
        
        return texture;
    }
    
    void Texture::build(unsigned int  flags,
                        unsigned char filter,
                        float		  anisotropic_filter)
    
    {
        char filename[ MAX_PATH ] = {""};
        
        sprintf(filename, "%s%s", fullPath, name);
        
        Memory *m = Memory::open(filename, 0);
        
        if (m){
            load(m);
            scale(nextPot(width), nextPot(height));
            
            generate_id(flags, filter, anisotropic_filter);
            free_texel_array();
            delete m;
        }
    }
    
    unsigned int  Texture::nextPot(unsigned int n)
    {
        n--;
        n |= n >> 1; n |= n >> 2;
        n |= n >> 4; n |= n >> 8;
        n |= n >> 16;
        n++;
        return n;
    }
    
    void Texture::load(Memory *memory)
    {
        char ext[MAX_CHAR] = {""};
        
        get_file_name(memory->filename, this->name);
        
        get_file_extension(memory->filename, ext, 1);
        
        if (!strcmp(ext, "PNG")) {
            load_png(memory);
        }
        else if (!strcmp(ext, "PVR")) {
            load_pvr(memory);
        }
    }
    
    /*!
     Callback function used internally to decompress a PNG file in memory.
     
     \param[in] structp The PNG structure that point to the attached MEMORY stream.
     \param[in] bytep The byte pointer to of the stream to read.
     \param[in] size The size in bytes to read from the stream.
     */
    void png_memory_read_callback(png_structp structp, png_bytep bytep, png_size_t size)
    {
        Memory *m = (Memory *)png_get_io_ptr(structp);
        m->read(bytep, (unsigned int)size);
    }
    
    void Texture::load_png(Memory *memory)
    {
        png_structp structp;
        
        png_infop infop;
        
        png_bytep *bytep = NULL;
        
        unsigned int i = 0;
        
        int n,
        png_bit_depth,
        png_color_type;
        
        structp = png_create_read_struct( PNG_LIBPNG_VER_STRING,
                                         NULL,
                                         NULL,
                                         NULL );
        
        infop = png_create_info_struct( structp );
        
        png_set_read_fn( structp, ( png_voidp * )memory, png_memory_read_callback );
        
        png_read_info( structp, infop );
        
        png_bit_depth = png_get_bit_depth( structp, infop );
        
        png_color_type = png_get_color_type( structp, infop );
        
        if( png_color_type == PNG_COLOR_TYPE_PALETTE )
        { png_set_expand( structp ); }
        
        if( png_color_type == PNG_COLOR_TYPE_GRAY && png_bit_depth < 8 )
        { png_set_expand( structp ); }
        
        if( png_get_valid( structp, infop, PNG_INFO_tRNS ) )
        { png_set_expand( structp ); }
        
        if( png_bit_depth == 16 )
        { png_set_strip_16( structp ); }
        
        if( png_color_type == PNG_COLOR_TYPE_GRAY || png_color_type == PNG_COLOR_TYPE_GRAY_ALPHA )
        { png_set_gray_to_rgb( structp ); }
        
        png_read_update_info( structp, infop );
        
        png_get_IHDR( structp,
                     infop,
                     ( png_uint_32 * )( &this->width  ),
                     ( png_uint_32 * )( &this->height ),
                     &png_bit_depth,
                     &png_color_type,
                     NULL, NULL, NULL );
        
        switch( png_color_type )
        {
            case PNG_COLOR_TYPE_GRAY:
            {
                this->byte			 = 1;
                this->internal_format =
                this->format		     = GL_LUMINANCE;
                
                break;
            }
                
            case PNG_COLOR_TYPE_GRAY_ALPHA:
            {
                this->byte			 = 2;
                this->internal_format =
                this->format		     = GL_LUMINANCE_ALPHA;
                
                break;
            }
                
            case PNG_COLOR_TYPE_RGB:
            {
                this->byte			 = 3;
                this->internal_format =
                this->format		     = GL_RGB;
                
                break;
            }
                
            case PNG_COLOR_TYPE_RGB_ALPHA:
            {
                this->byte			 = 4;
                this->internal_format =
                this->format		     = GL_RGBA;
                
                break;
            }
        }
        
        this->texel_type = GL_UNSIGNED_BYTE;
        
        this->size = this->width * this->height * this->byte;
        
        this->texel_array = ( unsigned char * ) malloc( this->size );
        
        bytep = ( png_bytep * ) malloc( this->height * sizeof( png_bytep ) );
        
        
        while( i != this->height )
        {
            n = this->height - ( i + 1 );
            
            bytep[ n ] = this->texel_array + ( n * this->width * this->byte );
            
            ++i;
        }
        
        
        png_read_image( structp, bytep );
        
        png_read_end( structp, NULL );
        
        png_destroy_read_struct( &structp,
                                &infop,
                                NULL );
        
        free( bytep );
    }
    
    void Texture::load_pvr(Memory *memory)
    {
        const unsigned char pvrtc_identifier[ 4 ] = { 'P', 'V', 'R', '!' };
        
        PVRHEADER *pvrheader = ( PVRHEADER * )memory->buffer;
        
        if( ( ( pvrheader->tag >> 0  ) & 0xFF ) != pvrtc_identifier[ 0 ] ||
           ( ( pvrheader->tag >> 8  ) & 0xFF ) != pvrtc_identifier[ 1 ] ||
           ( ( pvrheader->tag >> 16 ) & 0xFF ) != pvrtc_identifier[ 2 ] ||
           ( ( pvrheader->tag >> 24 ) & 0xFF ) != pvrtc_identifier[ 3 ] )
        { return; }
        
        
        if( ( pvrheader->flags & 0xFF ) == 24 || // PVRTC2
           ( pvrheader->flags & 0xFF ) == 25 )  // PVRTC4
        {
            this->width    = pvrheader->width;
            this->height   = pvrheader->height;
            this->byte     = pvrheader->bpp;
            this->n_mipmap = pvrheader->n_mipmap + 1;
            
            if( pvrheader->bitalpha )
            {
                this->compression = pvrheader->bpp == 4 ?
            GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG:
                GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG;
            }
            else
            {
                this->compression = pvrheader->bpp == 4 ?
            GL_COMPRESSED_RGB_PVRTC_4BPPV1_IMG:
                GL_COMPRESSED_RGB_PVRTC_2BPPV1_IMG;
            }
            
            this->texel_array = ( unsigned char * ) malloc( pvrheader->datasize );
            
            memcpy( this->texel_array,
                   &memory->buffer[ sizeof( PVRHEADER ) ],
                   pvrheader->datasize );
        }
    }
    
    void Texture::convert_16_bits(unsigned char use_5551)
    {
        unsigned int i = 0,
        s = this->width * this->height,
        *t = NULL;
        
        unsigned short *texel_array = NULL;
        
        switch( this->byte )
        {
            case 3:
            {
                unsigned int j = 0;
                
                unsigned char *tmp_array = NULL;
                
                this->byte		= 4;
                this->size		= s * this->byte;
                this->texel_type = GL_UNSIGNED_SHORT_5_6_5;
                
                tmp_array = ( unsigned char * ) malloc( this->size );
                
                while( i != this->size )
                {
                    tmp_array[ i     ] = this->texel_array[ j     ];
                    tmp_array[ i + 1 ] = this->texel_array[ j + 1 ];
                    tmp_array[ i + 2 ] = this->texel_array[ j + 2 ];
                    tmp_array[ i + 3 ] = 255;
                    
                    i += this->byte;
                    j += 3;
                }
                
                free( this->texel_array );
                this->texel_array = tmp_array;
                
                texel_array = ( unsigned short * )this->texel_array;
                
                t = ( unsigned int * )this->texel_array;
                
                i = 0;
                while( i != s )
                {
                    *texel_array++ = ( ( (   *t         & 0xff ) >> 3 ) << 11 ) |
                    ( ( ( ( *t >>  8 ) & 0xff ) >> 2 ) <<  5 ) |
                    ( ( *t >> 16 ) & 0xff ) >> 3;
                    ++t;
                    ++i;
                }
                
                break;
            }
                
            case 4:
            {
                texel_array = ( unsigned short * )this->texel_array;
                
                t = ( unsigned int * )this->texel_array;
                
                if( use_5551 )
                {
                    this->texel_type = GL_UNSIGNED_SHORT_5_5_5_1;
                    
                    while( i != s )
                    {
                        *texel_array++ = ( ( (   *t         & 0xff ) >> 3 ) << 11 ) |
                        ( ( ( ( *t >>  8 ) & 0xff ) >> 3 ) <<  6 ) |
                        ( ( ( ( *t >> 16 ) & 0xff ) >> 3 ) <<  1 ) |
                        ( ( *t >> 24 ) & 0xff ) >> 7;
                        ++t;
                        ++i;
                    }
                }
                else
                {
                    this->texel_type = GL_UNSIGNED_SHORT_4_4_4_4;
                    
                    while( i != s )
                    {
                        *texel_array++ = ( ( (   *t         & 0xff ) >> 4 ) << 12 ) |
                        ( ( ( ( *t >>  8 ) & 0xff ) >> 4 ) <<  8 ) |
                        ( ( ( ( *t >> 16 ) & 0xff ) >> 4 ) <<  4 ) |
                        ( ( *t >> 24 ) & 0xff ) >> 4;
                        ++t;
                        ++i;
                    }
                }
                
                break;
            }
        }
        
    }
    
    void Texture::generate_id(unsigned int flags, unsigned char filter, float anisotropic_filter)
    {
        if (this->tid) {
            delete_id();
        }
        
        glLog("glGenTextures");
        glGenTextures(1, &this->tid);
        glLog("glBindTexture");
        glBindTexture(this->target, this->tid);
        
        if( !this->compression )
        {
            switch( this->byte )
            {
                case 1: glPixelStorei( GL_PACK_ALIGNMENT, 1 ); break;
                case 2: glPixelStorei( GL_PACK_ALIGNMENT, 2 ); break;
                case 3:
                case 4: glPixelStorei( GL_PACK_ALIGNMENT, 4 ); break;
            }
            
            if( flags & TEXTURE_16_BITS ) {
                convert_16_bits(flags & TEXTURE_16_BITS_5551 );
            }
        }
        
        if( flags & TEXTURE_CLAMP )
        {
            glTexParameteri( this->target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
            glTexParameteri( this->target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
        }
        else
        {
            glTexParameteri( this->target, GL_TEXTURE_WRAP_S, GL_REPEAT );
            glTexParameteri( this->target, GL_TEXTURE_WRAP_T, GL_REPEAT );
        }
        
        
        if( anisotropic_filter )
        {
            static float texture_max_anisotropy = 0.0f;
            
            if( !texture_max_anisotropy ) glGetFloatv( GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT,
                                                      &texture_max_anisotropy );
            
            anisotropic_filter = CLAMP( anisotropic_filter,
                                       0.0f,
                                       texture_max_anisotropy );
            
            glTexParameterf( this->target,
                            GL_TEXTURE_MAX_ANISOTROPY_EXT,
                            anisotropic_filter );
        }
        
        
        if( flags & TEXTURE_MIPMAP )
        {
            switch( filter )
            {
                case TEXTURE_FILTER_1X:
                {
                    glTexParameteri( this->target, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR );
                    glTexParameteri( this->target, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
                    
                    break;
                }
                    
                case TEXTURE_FILTER_2X:
                {
                    glTexParameteri( this->target, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST );
                    glTexParameteri( this->target, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
                    
                    break;
                }
                    
                case TEXTURE_FILTER_3X:
                {
                    glTexParameteri( this->target, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
                    glTexParameteri( this->target, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
                    
                    break;
                }
                    
                case TEXTURE_FILTER_0X:
                default:
                {
                    glTexParameteri( this->target, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST );
                    glTexParameteri( this->target, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
                    
                    break;
                }
            }
        }
        else
        {
            switch( filter )
            {
                case TEXTURE_FILTER_0X:
                {
                    glTexParameteri( this->target, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
                    glTexParameteri( this->target, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
                    
                    break;
                }
                    
                default:
                {
                    glTexParameteri( this->target, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
                    glTexParameteri( this->target, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
                    
                    break;
                }
            }
        }
        
        
        if( this->compression )
        {
            unsigned int i		= 0,
            width  = this->width,
            height = this->height,
            size	= 0,
            offset = 0,
            bsize  = this->byte == 4 ? 16 : 32,
            bwidth,
            bheight;
            
            while( i != this->n_mipmap )
            {
                if( width  < 1 ){ width  = 1; }
                if( height < 1 ){ height = 1; }
                
                bwidth = this->byte == 4 ?
                width >> 2:
                width >> 3;
                
                bheight = height >> 2;
                
                size = bwidth * bheight * ( ( bsize * this->byte ) >> 3 );
                
                if( size < 32 ){ size = 32; }
                
                glCompressedTexImage2D( this->target,
                                       i,
                                       this->compression,
                                       width,
                                       height,
                                       0,
                                       size,
                                       &this->texel_array[ offset ] );
                width  >>= 1;
                height >>= 1;
                
                offset += size;
                
                ++i;
            }
        }
        else
        {
            glTexImage2D( this->target,
                         0,
                         this->internal_format,
                         this->width,
                         this->height,
                         0,
                         this->format,
                         this->texel_type,
                         this->texel_array );
        }
        
        
        if( flags & TEXTURE_MIPMAP && !this->n_mipmap ) {
            glGenerateMipmap( this->target );
        }
    }
    
    void Texture::delete_id()
    {
        if( this->tid )
        {
            glLog("glDeleteTextures");
            glDeleteTextures( 1, &this->tid );
            this->tid = 0;
        }
    }
    
    void Texture::free_texel_array()
    {
        if( this->texel_array )
        {
            free( this->texel_array );
            this->texel_array = NULL;
        }
    }
    
    void Texture::draw()
    {
        glLogVerbose("Texture::draw > glBindTexture");
        glBindTexture( this->target,
                      this->tid );
    }
    
    void Texture::scale(unsigned int width, unsigned int height)
    {
        unsigned int i = 0,
        j,
        offset,
        *px = ( unsigned int * ) malloc( width  << 3 ),
        *py = ( unsigned int * ) malloc( height << 3 );
        
        float t = 0.0f,
        ratio_x = ( float )this->width  / ( float )width,
        ratio_y = ( float )this->height / ( float )height;
        
        
        unsigned char *texel_array = ( unsigned char * ) malloc( width * height * this->byte );
        
        while( i != width )
        {
            px[ i ] = ( unsigned int )t;
            
            t += ratio_x;
            
            ++i;
        }
        
        
        i = 0;
        t = 0.0f;
        while( i != height )
        {
            py[ i ] = ( unsigned int )t;
            
            t += ratio_y;
            
            ++i;
        }
        
        
        i = 0;
        while( i != height )
        {
            offset = py[ i ] * this->width;
            
            j = 0;
            while( j != width )
            {
                switch( this->byte )
                {
                    case 1:
                    {
                        texel_array[ ( i * width + j ) ] = this->texel_array[ ( offset + px[ j ] ) ];
                        
                        break;
                    }
                        
                    case 2:
                    {
                        texel_array[ ( i * width + j ) * this->byte     ] = this->texel_array[ ( offset + px[ j ] ) * this->byte     ];
                        texel_array[ ( i * width + j ) * this->byte + 1 ] = this->texel_array[ ( offset + px[ j ] ) * this->byte + 1 ];
                        
                        break;
                    }
                        
                    case 3:
                    {
                        texel_array[ ( i * width + j ) * this->byte     ] = this->texel_array[ ( offset + px[ j ] ) * this->byte     ];
                        texel_array[ ( i * width + j ) * this->byte + 1 ] = this->texel_array[ ( offset + px[ j ] ) * this->byte + 1 ];
                        texel_array[ ( i * width + j ) * this->byte + 2 ] = this->texel_array[ ( offset + px[ j ] ) * this->byte + 2 ];
                        
                        break;
                    }
                        
                    case 4:
                    {
                        texel_array[ ( i * width + j ) * this->byte     ] = this->texel_array[ ( offset + px[ j ] ) * this->byte     ];
                        texel_array[ ( i * width + j ) * this->byte + 1 ] = this->texel_array[ ( offset + px[ j ] ) * this->byte + 1 ];
                        texel_array[ ( i * width + j ) * this->byte + 2 ] = this->texel_array[ ( offset + px[ j ] ) * this->byte + 2 ];
                        texel_array[ ( i * width + j ) * this->byte + 3 ] = this->texel_array[ ( offset + px[ j ] ) * this->byte + 3 ];
                        
                        break;
                    }
                }
                
                ++j;
            }
            
            
            ++i;
        }
        
        
        free( px );
        free( py );
        
        px =
        py = NULL;
        
        this->width  = width;
        this->height = height;
        
        free( this->texel_array );
        this->texel_array = texel_array;
    }
    
}
