#include "berry.h"
#include <stdio.h>
#include <string.h>

void be_writebuffer(const char *buffer, size_t length)
{
    fwrite(buffer, 1, length, stdout);
}

char* be_readstring(char *buffer, size_t size)
{
    return fgets(buffer, (int)size, stdin);
}

void* be_fopen(const char *filename, const char *modes)
{
    return fopen(filename, modes);
}

int be_fclose(void *hfile)
{
    return fclose((FILE*)hfile);
}

size_t be_fwrite(void *hfile, const void *buffer, size_t length)
{
    return fwrite(buffer, 1, length, (FILE*)hfile);
}

size_t be_fread(void *hfile, void *buffer, size_t length)
{
    return fread(buffer, 1, length, (FILE*)hfile);
}

char* be_fgets(void *hfile, void *buffer, int size)
{
    return fgets((char*)buffer, size, (FILE*)hfile);
}

int be_fseek(void *hfile, long offset, int origin)
{
    return fseek((FILE*)hfile, offset, origin);
}

long be_ftell(void *hfile)
{
    return ftell((FILE*)hfile);
}

long be_fsize(void *hfile)
{
    long size, cur = ftell((FILE*)hfile);
    fseek((FILE*)hfile, 0, SEEK_END);
    size = ftell((FILE*)hfile);
    fseek((FILE*)hfile, cur, SEEK_SET);
    return size;
}

int be_fflush(void *hfile)
{
    return fflush((FILE*)hfile);
}

#include <stdarg.h>
void berry_log_C(const char * berry_buf, ...)
{
    va_list args;
    va_start(args, berry_buf);
    vfprintf(stderr, berry_buf, args);
    va_end(args);
    fprintf(stderr, "\n");
}
