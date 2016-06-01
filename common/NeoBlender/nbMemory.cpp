//
//  flMemory.cpp
//  NeoBlender
//
//  Created by Nam, SangWook on 2015. 1. 29..
//  Copyright (c) 2015년 flowgrammer. All rights reserved.
//

#include "nbMemory.h"
#include <iostream>
#include <string>
#include <fstream>
#include "nbHeader.h"


using namespace std;

namespace nbd {

Memory::~Memory()
{
    close();
}

Memory *Memory::open(char *filename, unsigned char relative_path)
{
#if 1
    char fname[MAX_PATH] = {""};
    
    if (relative_path) {
        get_file_path( getenv( "FILESYSTEM" ), fname );
        strcat(fname, filename);
    }
    else {
        strcpy(fname, filename);
    }
    
    ifstream file;
    file.open(fname, ios::binary);
    
    if (!file.good()) {
        cout << "error opening file, " << fname << endl;
        return NULL;
    }
    
    Memory *memory = new Memory();
    
    strcpy(memory->filename, fname);
    
    unsigned int begin = (unsigned int)file.tellg();
    file.seekg(0, ios::end);
    unsigned int end = (unsigned int)file.tellg();
    file.seekg(0, ios::beg);
    memory->size = end - begin;
    
    memory->buffer = (unsigned char *)calloc(1, memory->size + 1);
    
    file.read((char *)memory->buffer, memory->size);
    file.close();
    
    return memory;
#else
    char fname[MAX_PATH] = {""};
    
    if (relative_path) {
        get_file_path( getenv( "FILESYSTEM" ), fname );
        strcat(fname, filename);
    }
    else {
        strcpy(fname, filename);
    }
    
    FILE *f = fopen(fname, "rb");
    
    if (!f) {
        return NULL;
    }
    
    Memory *memory = new Memory();
    
    strcpy(memory->filename, fname);
    
    fseek(f, 0, SEEK_END);
    memory->size = (unsigned int)ftell(f);
    fseek(f, 0, SEEK_SET);
    
    memory->buffer = (unsigned char *)calloc(1, memory->size + 1);
    fread(memory->buffer, memory->size, 1, f);
    memory->buffer[memory->size] = 0;
    
    fclose(f);
    
    return memory;
    
#endif
}

void Memory::close()
{
    if(buffer) {
        free(buffer);
    }
}

unsigned int Memory::read(void *dst, unsigned int size )
{
    if ((this->position + size) > this->size) {
        size = this->size - this->position;
    }
    
    memcpy(dst, &this->buffer[this->position], size );
    
    this->position += size;
    
    return size;
}

void Memory::insert(char *str, unsigned int position )
{
    unsigned int s1 = (unsigned int)strlen(str),
    s2 = this->size + s1 + 1;
    
    char *buffer = (char *)this->buffer,
    *tmp	 = (char *)calloc( 1, s2 );
    
    if (position) {
        strncpy(&tmp[0], &buffer[0], position);
    }
    
    strcat(&tmp[position], str);
    
    strcat(&tmp[position + s1], &buffer[position]);
    
    this->size = s2;
    
    free(this->buffer);
    this->buffer = (unsigned char *)tmp;
}
    
}

