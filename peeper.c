/*
 * Based on the V4L2 video capture example at
 * http://linuxtv.org/docs.php .
 *
 * Works with v4l2 compatible webcam. (Not v4l.)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>

#include "jpeglib.h"


#include <linux/videodev2.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))


enum io_method {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

struct buffer {
        void   *start;
        size_t  length;
};

static char            *dev_name;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;

// Command line flags
static int              force_format;
static int              scale = 16;
static float            threshold = 0.1;

static int 			    capture_width = 0;
static int			    capture_height = 0;

static unsigned char*   last_buf;               // Last sucessfull read from webcam.
static unsigned char*   simplified_buf;         // Webcam monochrome image scalled down by scale.
static float*           average_buf;            // Average monochrome image over last several frames.
static unsigned char*   movment_buf;            // Diff between simplified_buf and average_buf.
static unsigned char*   rgb_buf;                // last_buf converted to RGB colours.

/**
  Convert from YUV422 format to RGB888. Formulae are described on http://en.wikipedia.org/wiki/YUV
  http://www.twam.info/linux/v4l2grab-grabbing-jpegs-from-v4l2-devices

  \param width width of image
  \param height height of image
  \param src source
  \param dst destination
*/
static void YUV422toRGB888(int width, int height, unsigned char *src, unsigned char *dst)
{
  int line, column;
  unsigned char *py, *pu, *pv;
  unsigned char *tmp = dst;

  /* In this format each four bytes is two pixels. Each four bytes is two Y's, a Cb and a Cr. 
     Each Y goes to one of the pixels, and the Cb and Cr belong to both pixels. */
  py = src;
  pu = src + 1;
  pv = src + 3;

  #define CLIP(x) ( (x)>=0xFF ? 0xFF : ( (x) <= 0x00 ? 0x00 : (x) ) )

  for (line = 0; line < height; ++line) {
    for (column = 0; column < width; ++column) {
      *tmp++ = CLIP((double)*py + 1.402*((double)*pv-128.0));
      *tmp++ = CLIP((double)*py - 0.344*((double)*pu-128.0) - 0.714*((double)*pv-128.0));
      *tmp++ = CLIP((double)*py + 1.772*((double)*pu-128.0));

      // increase py every time
      py += 2;
      // increase pu,pv every second time
      if ((column & 1)==1) {
        pu += 4;
        pv += 4;
      }
    }
  }
}

static void write_JPEG_file (unsigned char* p_image_buffer, char* filename, int image_width, int image_height, int num_of_col)
{
    // JPEG object
    struct jpeg_compress_struct cinfo;

    // JPEG error handler
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);

    FILE * outfile;               /* target file */
    JSAMPROW row_pointer[1];      /* pointer to JSAMPLE row[s] */

    // initialize the JPEG compression object.
    jpeg_create_compress(&cinfo);

    // open file and set file as target.
    if ((outfile = fopen(filename, "wb")) == NULL) {
        fprintf(stderr, "can't open %s\n", filename);
        exit(1);
    }
    jpeg_stdio_dest(&cinfo, outfile);

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

    // set any non default cinfo oeramiters.
    jpeg_set_quality(&cinfo, 70, TRUE /* limit to baseline-JPEG values */);

    jpeg_start_compress(&cinfo, TRUE);
    int row_stride = image_width * num_of_col;   // 3 values per pixel.
    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0] = &p_image_buffer[cinfo.next_scanline * row_stride];
        (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    fclose(outfile);
    jpeg_destroy_compress(&cinfo);
}

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
        int r;

        do {
                r = ioctl(fh, request, arg);
        } while (-1 == r && EINTR == errno);

        return r;
}

static void init_buf()
{
    simplified_buf = malloc(sizeof(unsigned char) * capture_width * capture_height / (scale * scale));
    if (!simplified_buf) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }
    average_buf = malloc(sizeof(float) * capture_width * capture_height / (scale * scale));
    if (!average_buf) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }
    movment_buf = malloc(sizeof(unsigned char) * capture_width * capture_height / (scale * scale));
    if (!movment_buf) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }
    rgb_buf = malloc(sizeof(unsigned char) * capture_width * capture_height * 3);
    if (!rgb_buf) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

}

static void uninit_buf()
{
    free(simplified_buf);
    free(average_buf);
    free(movment_buf);
    free(rgb_buf);
}

static void process_image(const void *p, int size)
{
    last_buf = (unsigned char*)p;
    unsigned char *py, *pu, *pv;
    int row, colum;
    int val;
    unsigned char *tmp = simplified_buf;
    py = (unsigned char*)p;
    pu = (unsigned char*)p + 1;
    pv = (unsigned char*)p + 3;
    for(row = 0; row < capture_height; ++row){
        for(colum = 0; colum < capture_width; ++colum){
            if (!(colum % scale) & !(row % scale)) {
                // copy pixel into buffer
                val = *py;  // + *pu + *pv;
                *tmp = val;
                tmp++;
            }
            // increase py every time
            py += 2;
            // increase pu,pv every second time
            if ((colum & 1)==1) {
                pu += 4;
                pv += 4;
            }
        }
    }
}

static void update_movment()
{
    int row, colum;
    unsigned char *tmp_last = simplified_buf;
    float *tmp_average = average_buf;
    unsigned char *tmp_movment = movment_buf;

    for(row = 0; row < capture_height; row += scale){
        for(colum = 0; colum < capture_width; colum += scale){
            *tmp_movment = abs(*tmp_last - *tmp_average);

            if ((*tmp_last > *tmp_average) & (*tmp_average < 255)) {
                *tmp_average += threshold;
            } else if ((*tmp_last < *tmp_average) & (*tmp_average > 0)) {
                *tmp_average -= threshold;
            }

            tmp_last++;
            tmp_average++;
            tmp_movment++;
        }
    }
}

static void display_image(void *p_buffer)
{
    int row, colum;
    int val;
    unsigned char *tmp = p_buffer;
    
    fprintf(stderr, "\n+");
    for(colum = 0; colum < capture_width; colum += scale){
        fprintf(stderr, "--");
    }
    fprintf(stderr, "+\n|");
    for(row = 0; row < capture_height; row += scale){
        if (row) fprintf(stderr, "|\n|");
        for(colum = 0; colum < capture_width; colum += scale){
            val = *tmp;
            tmp++;
            if (val < 20) {
                fprintf(stderr, "  ");
            } else if ( val < 40) {
                fprintf(stderr, "..");
            } else if ( val < 60) {
                fprintf(stderr, "--");
            } else if ( val < 80) {
                fprintf(stderr, "~~");
            } else if ( val < 100) {
                fprintf(stderr, "**");
            } else if ( val < 150) {
                fprintf(stderr, "xx");
            } else if ( val < 200) {
                fprintf(stderr, "XX");
            } else {
                fprintf(stderr, "##");
            }
        }
    }
    fprintf(stderr, "|\n+");
    for(colum = 0; colum < capture_width; colum += scale){
        fprintf(stderr, "--");
    }
    fprintf(stderr, "+\n");
}

static int read_frame(void)
{
        struct v4l2_buffer buf;
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */

                        default:
                                errno_exit("read");
                        }
                }

                process_image(buffers[0].start, buffers[0].length);
                break;

        case IO_METHOD_MMAP:
                CLEAR(buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */

                        default:
                                errno_exit("VIDIOC_DQBUF");
                        }
                }

                assert(buf.index < n_buffers);

                process_image(buffers[buf.index].start, buf.bytesused);

                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                        errno_exit("VIDIOC_QBUF");
                break;

        case IO_METHOD_USERPTR:
                CLEAR(buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;

                if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */

                        default:
                                errno_exit("VIDIOC_DQBUF");
                        }
                }

                for (i = 0; i < n_buffers; ++i)
                        if (buf.m.userptr == (unsigned long)buffers[i].start
                            && buf.length == buffers[i].length)
                                break;

                assert(i < n_buffers);

                process_image((void *)buf.m.userptr, buf.bytesused);

                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                        errno_exit("VIDIOC_QBUF");
                break;
        }

        return 1;
}

static void mainloop(void)
{

    for (;;) {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r) {
            if (EINTR == errno)
                continue;
            errno_exit("select");
        }

        if (0 == r) {
            fprintf(stderr, "select timeout\n");
            exit(EXIT_FAILURE);
        }

        if (read_frame())
            break;
        /* EAGAIN - continue select loop. */
    }
}

static void stop_capturing(void)
{
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                        errno_exit("VIDIOC_STREAMOFF");
                break;
        }
}

static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_USERPTR;
                        buf.index = i;
                        buf.m.userptr = (unsigned long)buffers[i].start;
                        buf.length = buffers[i].length;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;
        }
}

static void uninit_device(void)
{
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                free(buffers[0].start);
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap(buffers[i].start, buffers[i].length))
                                errno_exit("munmap");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i)
                        free(buffers[i].start);
                break;
        }

        free(buffers);
}

static void init_read(unsigned int buffer_size)
{
        buffers = calloc(1, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);

        if (!buffers[0].start) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
}

static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) {
                fprintf(stderr, "Insufficient buffer memory on %s\n",
                         dev_name);
                exit(EXIT_FAILURE);
        }

        buffers = calloc(req.count, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

static void init_userp(unsigned int buffer_size)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = 4;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                 "user pointer i/o\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        buffers = calloc(4, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = malloc(buffer_size);

                if (!buffers[n_buffers].start) {
                        fprintf(stderr, "Out of memory\n");
                        exit(EXIT_FAILURE);
                }
        }
}

static void init_device(void)
{
        struct v4l2_capability cap;
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        struct v4l2_format fmt;
        unsigned int min;

        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s is no V4L2 device\n",
                                 dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_QUERYCAP");
                }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                fprintf(stderr, "%s is no video capture device\n",
                         dev_name);
                exit(EXIT_FAILURE);
        }

        switch (io) {
        case IO_METHOD_READ:
                if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
                        fprintf(stderr, "%s does not support read i/o\n",
                                 dev_name);
                        exit(EXIT_FAILURE);
                }
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                        fprintf(stderr, "%s does not support streaming i/o\n",
                                 dev_name);
                        exit(EXIT_FAILURE);
                }
                break;
        }


        /* Select video input, video standard and tune here. */


        CLEAR(cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect; /* reset to default */

                if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
                        switch (errno) {
                        case EINVAL:
                                /* Cropping not supported. */
                                break;
                        default:
                                /* Errors ignored. */
                                break;
                        }
                }
        } else {
                /* Errors ignored. */
        }


        CLEAR(fmt);

        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (force_format) {
                fmt.fmt.pix.width       = 640;
                fmt.fmt.pix.height      = 480;
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
                fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

                if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                        errno_exit("VIDIOC_S_FMT");

                /* Note VIDIOC_S_FMT may change width and height. */
        } else {
                /* Preserve original settings as set by v4l2-ctl for example */
                if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                        errno_exit("VIDIOC_G_FMT");
        }

        /* Buggy driver paranoia. */
        min = fmt.fmt.pix.width * 2;
        if (fmt.fmt.pix.bytesperline < min)
                fmt.fmt.pix.bytesperline = min;
        min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        if (fmt.fmt.pix.sizeimage < min)
                fmt.fmt.pix.sizeimage = min;

        switch (io) {
        case IO_METHOD_READ:
                init_read(fmt.fmt.pix.sizeimage);
                break;

        case IO_METHOD_MMAP:
                init_mmap();
                break;

        case IO_METHOD_USERPTR:
                init_userp(fmt.fmt.pix.sizeimage);
                break;
        }

	capture_width = fmt.fmt.pix.width;
	capture_height = fmt.fmt.pix.height;
    fprintf(stderr,"Image width set to %i by device %s.\n", capture_width, dev_name);
    fprintf(stderr,"Image height set to %i by device %s.\n", capture_height, dev_name);
}

static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

static void open_device(void)
{
        struct stat st;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

static void usage(FILE *fp, int argc, char **argv)
{
        fprintf(fp,
                 "Usage: %s [options]\n\n"
                 "Version 1.3\n"
                 "Options:\n"
                 "-d | --device name   Video device name [%s]\n"
                 "-h | --help          Print this message\n"
                 "-m | --mmap          Use memory mapped buffers [default]\n"
                 "-r | --read          Use read() calls\n"
                 "-u | --userp         Use application allocated buffers\n"
                 "-f | --format        Force format to 640x480 YUYV\n"
                 "-s | --scale         Divide raw webcam image by this value [%i]\n"
                 "-t | --threshold     Rate at which changes in image are absorbed into the expected backround [%f]\n"
                 "",
                 argv[0], dev_name, scale, threshold);
}

static const char short_options[] = "d:hmruofs:t:";

static const struct option
long_options[] = {
        { "device", required_argument, NULL, 'd' },
        { "help",   no_argument,       NULL, 'h' },
        { "mmap",   no_argument,       NULL, 'm' },
        { "read",   no_argument,       NULL, 'r' },
        { "userp",  no_argument,       NULL, 'u' },
        { "format", no_argument,       NULL, 'f' },
        { "scale",  required_argument, NULL, 's' },
        { "threshold", required_argument, NULL, 't' },
        { 0, 0, 0, 0 }
};

int main(int argc, char **argv)
{
    clock_t begin, end;

    dev_name = "/dev/video0";

    for (;;) {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c) {
            case 0: /* getopt_long() flag */
                break;

            case 'd':
                dev_name = optarg;
                break;

            case 'h':
                usage(stdout, argc, argv);
                exit(EXIT_SUCCESS);

            case 'm':
                io = IO_METHOD_MMAP;
                break;

            case 'r':
                io = IO_METHOD_READ;
                break;

            case 'u':
                io = IO_METHOD_USERPTR;
                break;

            case 'f':
                force_format++;
                break;
            
            case 's':
                scale = atoi(optarg);
                break;

            case 't':
                threshold = atoi(optarg);
                break;

            default:
                usage(stderr, argc, argv);
                exit(EXIT_FAILURE);
        }
    }

    open_device();
    init_device();
    init_buf();
    start_capturing();
    begin = clock();
    while (1) {
        mainloop();
        end = clock();
        //printf("%d %d %d %f\n", begin, end, CLOCKS_PER_SEC, (float)(end - begin) / CLOCKS_PER_SEC );
        if ((float)(end - begin) / CLOCKS_PER_SEC > .01) {
            begin = clock();
            update_movment();
            display_image(movment_buf);

            YUV422toRGB888(capture_width, capture_height, last_buf, rgb_buf);
            write_JPEG_file(rgb_buf, "test.jpeg", capture_width, capture_height, 3);
            write_JPEG_file(movment_buf, "test2.jpeg", capture_width / scale, capture_height / scale, 1);
        }
    }
    stop_capturing();
    uninit_buf();
    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}
