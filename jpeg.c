#include <stdio.h>
#include <stdlib.h>

#include "jpeglib.h"

#include "jpeg.h"


void strreverse(char* begin, char* end) {
    char aux;
    while(end>begin)
        aux=*end, *end--=*begin, *begin++=aux;
}

void itoa(int value, char* str, int base) {
    static char num[] = "0123456789abcdefghijklmnopqrstuvwxyz";
    char* wstr=str;
    int sign;

    // Validate base
    if (base<2 || base>35){ *wstr='\0'; return; }

    // Take care of sign
    if ((sign=value) < 0) value = -value;

    // Conversion. Number is reversed.
    do *wstr++ = num[value%base]; while(value/=base);
    if(sign<0) *wstr++='-';
    *wstr='\0';

    // Reverse string
    strreverse(str,wstr-1);
}

unsigned long jpeg_file_size(char* filename){
    FILE* fp = fopen(filename, "rb");
    if (!fp){
        fprintf(stderr,"ERROR, could not read file.\n");
        return 0;
    }

    //Get file length
    fseek(fp, 0, SEEK_END);
    unsigned long fileLen=ftell(fp);
    fseek(fp, 0, SEEK_SET);

    fclose(fp);

    return fileLen;
}

int jpeg_file_get(char* filename, char *buffer, unsigned long fileLen){
    FILE* fp;
    fp = fopen(filename, "rb");
    if (!fp){
        fprintf(stderr,"ERROR, could not read file.\n");
        return -1;
    }

    long int read_len = fread(buffer, sizeof(char), fileLen, fp);
    fprintf(stderr, "read %li bytes out of %li\n\n", read_len, fileLen);
    fclose(fp);

    return read_len;
}

void write_JPEG_file(char* filename, unsigned char* p_image_buffer, int image_width, int image_height, int num_of_col)
{
    // JPEG object
    struct jpeg_compress_struct cinfo;

    // JPEG error handler
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);

    JSAMPROW row_pointer[1];      /* pointer to JSAMPLE row[s] */

    // initialize the JPEG compression object.
    jpeg_create_compress(&cinfo);

    // open file and set file as target.
    FILE * outfile;               /* target file */
    if ((outfile = fopen(filename, "wb")) == NULL) {
        fprintf(stderr, "can't open %s\n", filename);
        exit(1);
    }
    jpeg_stdio_dest(&cinfo, outfile);
    //unsigned char *mem = NULL;
    //unsigned long mem_size = 0;
    //jpeg_mem_dest(&cinfo, &mem, &mem_size);

    cinfo.image_width = image_width;      /* image width and height, in pixels */
    cinfo.image_height = image_height;
    cinfo.input_components = num_of_col;  /* # of color components per pixel */
    if (num_of_col == 3){
        cinfo.in_color_space = JCS_RGB;       /* colorspace of input image */
    } else {
        cinfo.in_color_space = JCS_GRAYSCALE;
    }

    // set other cinfo peramiters as default.
    jpeg_set_defaults(&cinfo);

    // set any non default cinfo peramiters.
    jpeg_set_quality(&cinfo, 70, TRUE /* limit to baseline-JPEG values */);

    jpeg_start_compress(&cinfo, TRUE);
    int row_stride = image_width * num_of_col;
    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0] = &p_image_buffer[cinfo.next_scanline * row_stride];
        (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    fclose(outfile);
    //free(mem);
    jpeg_destroy_compress(&cinfo);
}
