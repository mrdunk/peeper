#ifndef JPEG_H
#define JPEG_H

#include <stdio.h>


/* itoa: Convert int to char*
 * Arguments:
 *      (int)value: int to conveert.
 *      (char*)str: Buffer to hold string. (Null terminated after conversion.)
 *      (int)base:  Base of number. (eg. 1 for bunary, 10 for base10.)
 */
void itoa(int value, char* str, int base);

/* jpeg_file_size: Get size (in bytes) of Jpeg file.
 * Arguments:
 *      (char*)filename: Pointer to name of file.
 */
unsigned long jpeg_file_size(char* filename);

/* jpeg_file_get: Load Jpeg file into buffer.
 * Arguments:
 *      (char*)filename: Pointer to name of file.
 *      (char*)buffer:   Buffer to be populated with Jpeg data.
 *      (unsigned long)fileLen: Expected length of file.
 * Returns:
 *      (int): -1 == failure.
 *              Otherwise, returns number of bytes read from file.
 */
int jpeg_file_get(char* filename, char *buffer, unsigned long fileLen);

void write_JPEG_file(char* filename, unsigned char* p_image_buffer, int image_width, int image_height, int num_of_col);

#endif  // JPEG_H
