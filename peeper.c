/*
 * Based on the V4L2 video capture example at
 * http://linuxtv.org/downloads/v4l-dvb-apis/capture-example.html
 *
 * Works with v4l2 compatible webcam. (Not v4l.)
 *
 * $ gcc ./peeper.c ./jpeg.c -ljpeg -lrt -Wall
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

#include "jpeg.h"

#include <linux/videodev2.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define BILLION  1000000000L

#define R 0
#define G 1
#define B 2

enum io_method {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

struct buffer {
        void   *start;
        size_t  length;
};

struct screen_buf {
        void*           start;
        size_t          length;
        int             width;
        int             height;
        pthread_mutex_t lock;
};

static struct screen_buf last_frame;

static char            *dev_name;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;

// Command line flags
static int              force_format;
static int              scale = 16;
static float            ave_thresh = 0.1;
static int              bright_thresh = 20;
static int              col_thresh = 10;

static int 			    capture_width = 0;
static int			    capture_height = 0;
static int              first_run = 1;

// Data containers
//static unsigned char*   last_buf;               // Last sucessfull read from webcam.
static float*           average_buf;            // Average monochrome image over last several frames.
static unsigned char*   average_char_buf;       // unsigned char buffer with average_buf data in it.
static unsigned char*   movment_buf;            // Diff between rgb_buf and average_buf.
//static unsigned char*   rgb_buf;                // last_buf converted to RGB colours.


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

static void float_buf_to_char_buf(float* float_buf, unsigned char* char_buf, int image_width, int image_height, int num_of_col)
{
    int i = 0;
    while (i < image_width * image_height * num_of_col) {
        if (float_buf[i] > 255){
            char_buf[i] = 255;
        } else if (float_buf[i] < 0) {
            char_buf[i] = 0;
        } else {
            char_buf[i] = float_buf[i];
        }
        i++;
    }
}

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static void process_image(const void *p, int size)
{   
    // Save pointer to last sucessfully filled v4l2 buffer.
    last_frame.start = (void*)p;
    last_frame.width = capture_width;
    last_frame.height = capture_height;
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
    average_buf = malloc(sizeof(float) * 3 * capture_width * capture_height / (scale * scale));
    if (!average_buf) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }
    average_char_buf = malloc(sizeof(unsigned char) * 3 * capture_width * capture_height / (scale * scale));
    if (!average_char_buf) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }
    movment_buf = malloc(sizeof(unsigned char) * capture_width * capture_height / (scale * scale));
    if (!movment_buf) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }
//    rgb_buf = malloc(sizeof(unsigned char) * capture_width * capture_height * 3);
//    if (!rgb_buf) {
//        fprintf(stderr, "Out of memory\n");
//        exit(EXIT_FAILURE);
//    }

}

static void uninit_buf()
{
    free(average_buf);
    free(average_char_buf);
    free(movment_buf);
//    free(rgb_buf);
}

static void update_movment(unsigned char* _rgb_source_buf) {
    int row, colum;
    float *tmp_average = average_buf;
    unsigned char *tmp_movment = movment_buf;

    for(row = 0; row < capture_height; row++){
        for(colum = 0; colum < capture_width; colum++){
            if (!(row % scale) & !(colum % scale)) {
                if (first_run) {
                    // Copy the first frame into the average buffer.
                    tmp_average[R] = _rgb_source_buf[R];
                    tmp_average[G] = _rgb_source_buf[G];
                    tmp_average[B] = _rgb_source_buf[B];
                } else {
                    // Slowly change the average buffer to match what is seen by the camera.
                    if ((_rgb_source_buf[R] > tmp_average[R]) & (tmp_average[R] < 255)) {
                        tmp_average[R] += ave_thresh;
                    } else if ((_rgb_source_buf[R] < tmp_average[R]) & (tmp_average[R] > 0)) {
                        tmp_average[R] -= ave_thresh;
                    }
                    if ((_rgb_source_buf[G] > tmp_average[G]) & (tmp_average[G] < 255)) {
                        tmp_average[G] += ave_thresh;
                    } else if ((_rgb_source_buf[G] < tmp_average[G]) & (tmp_average[G] > 0)) {
                        tmp_average[G] -= ave_thresh;
                    }
                    if ((_rgb_source_buf[B] > tmp_average[B]) & (tmp_average[B] < 255)) {
                        tmp_average[B] += ave_thresh;
                    } else if ((_rgb_source_buf[B] < tmp_average[B]) & (tmp_average[B] > 0)) {
                        tmp_average[B] -= ave_thresh;
                    }

                    // difference between the average image and the current one for each colour.
                    int r_diff = _rgb_source_buf[R] - tmp_average[R];
                    int g_diff = _rgb_source_buf[G] - tmp_average[G];
                    int b_diff = _rgb_source_buf[B] - tmp_average[B];

                    // difference between the colours.
                    // if all colours get brighter (or dimmer) by the same about, then val == 0.
                    // only if some colours change more than others do we register a change.
                    int col_change = abs(r_diff - g_diff) + abs(g_diff - b_diff) + abs(b_diff - r_diff);
                    if (col_change > 255) { col_change = 255; }

                    // difference in brightness of all 3 colours combined.
                    int bright_change = abs(_rgb_source_buf[R] + _rgb_source_buf[G] + _rgb_source_buf[B] -
                                            tmp_average[R] - tmp_average[G] - tmp_average[B]) / 3;

                    if (col_change > col_thresh && bright_change > bright_thresh) {
                        //*tmp_movment = col_change;
                        *tmp_movment = (_rgb_source_buf[R] + _rgb_source_buf[G] + _rgb_source_buf[B]) / 3;
                    } else {
                        *tmp_movment = 0;
                    }
                }
                tmp_average += 3;
                tmp_movment++;
            }
            _rgb_source_buf += 3;
        }
    }
}

#define MAXSIZE 16
static void display_image(void *p_buffer)
{
    int row, colum;
    int val;
    unsigned char *tmp = p_buffer;
    
    //int scale_remainder = 0;
    //if (scale < MAXSIZE) {
    //    scale_remainder = MAXSIZE - scale;
    //}

    fprintf(stderr, "\n+");
    for(colum = 0; colum < capture_width; colum += scale){
        if (!(colum % MAXSIZE)) {
            fprintf(stderr, "--");
        }
    }
    fprintf(stderr, "+\n|");
    for(row = 0; row < capture_height; row += scale){
        if (!(row % MAXSIZE)) {
            if (row) fprintf(stderr, "|\n|");
        }
        for(colum = 0; colum < capture_width; colum += scale){
            if (!(colum % MAXSIZE) & !(row % MAXSIZE)) {
                val = *tmp;
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
            tmp++;
        }
    }
    fprintf(stderr, "|\n+");
    for(colum = 0; colum < capture_width; colum += scale){
        if (!(colum % MAXSIZE)) {
            fprintf(stderr, "--");
        }
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
        struct v4l2_control control;
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

    // Turn off anything that might auto-adjust the brightness/contrast.
    // "$ v4l2-ctl -l" lets us see what our camera is capable of (and set to).
    memset (&control, 0, sizeof (control));
    control.id = V4L2_CID_AUTO_WHITE_BALANCE;
    control.value = 0;
    ioctl (fd, VIDIOC_S_CTRL, &control);    // Errors ignored
    memset (&control, 0, sizeof (control));
    control.id = V4L2_CID_RED_BALANCE;
    control.value = 0;
    ioctl (fd, VIDIOC_S_CTRL, &control);    // Errors ignored
    memset (&control, 0, sizeof (control));
    control.id = V4L2_CID_BLUE_BALANCE;
    control.value = 0;
    ioctl (fd, VIDIOC_S_CTRL, &control);    // Errors ignored
    memset (&control, 0, sizeof (control));
    control.id = V4L2_CID_AUTOGAIN;
    control.value = 0;
    ioctl (fd, VIDIOC_S_CTRL, &control);    // Errors ignored
    memset (&control, 0, sizeof (control));
    control.id = V4L2_CID_HUE_AUTO;
    control.value = 0;
    ioctl (fd, VIDIOC_S_CTRL, &control);    // Errors ignored
    memset (&control, 0, sizeof (control));
    control.id =  V4L2_CID_BACKLIGHT_COMPENSATION;
    control.value = 0;
    ioctl (fd, VIDIOC_S_CTRL, &control);    // Errors ignored
    memset (&control, 0, sizeof (control));
    control.id =  V4L2_EXPOSURE_AUTO;
    control.value = 3;
    ioctl (fd, VIDIOC_S_CTRL, &control);    // Errors ignored
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

void get_rgb(struct screen_buf* rgb_out){
    if(rgb_out->length == 0){
        rgb_out->start = malloc(sizeof(unsigned char) * last_frame.width * last_frame.height * 3);
    } else if(rgb_out->length < sizeof(unsigned char) * last_frame.width * last_frame.height * 3){
        rgb_out->length = sizeof(unsigned char) * last_frame.width * last_frame.height * 3;
        rgb_out->start = realloc(rgb_out->start, rgb_out->length);
    }
    if(!rgb_out->start){
        fprintf(stderr, "Memory error!");
        exit(1);
    }
    rgb_out->width = last_frame.width;
    rgb_out->height = last_frame.height;
    
    YUV422toRGB888(last_frame.width, last_frame.height, last_frame.start, rgb_out->start);    
}

void get_jpeg(){

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
                 "-s | --scale         Raw image devided by this scale [%i]\n"
                 "-a | --ave_thresh    Rate at which changes in image are absorbed into the expected backround [%f]\n"
                 "-b | --bright_thresh Sensitivity to movment. 0 = high sensitivity. 255 = no sensitivity [%i]\n"
                 "                     Lower this if contrast is bad but colours are different.\n"
                 "-c | --col_thresh    Sensitivity to movment. 0 = high sensitivity. 255 = no sensitivity [%i]\n"
                 "                     Lower this if detected colours are similar to background.\n"
                 "",
                 argv[0], dev_name, scale, ave_thresh, bright_thresh, col_thresh);
}

static const char short_options[] = "d:hmruofs:a:b:c:";

static const struct option
long_options[] = {
        { "device", required_argument, NULL, 'd' },
        { "help",   no_argument,       NULL, 'h' },
        { "mmap",   no_argument,       NULL, 'm' },
        { "read",   no_argument,       NULL, 'r' },
        { "userp",  no_argument,       NULL, 'u' },
        { "format", no_argument,       NULL, 'f' },
        { "scale",  required_argument, NULL, 's' },
        { "ave_thresh", required_argument, NULL, 'a' },
        { "bright_thresh", required_argument, NULL, 'b' },
        { "col_thresh", required_argument, NULL, 'c' },
        { 0, 0, 0, 0 }
};

int main(int argc, char **argv)
{
    struct timespec begin, end;

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
                if (!((scale == 1) | (scale == 2) | (scale == 4) | (scale == 8) | (scale == 16) |
                      (scale == 32) | (scale == 64) | (scale == 128))) {
                    fprintf(stderr, "--scale must be one of [1,2,4,8,16,32,64,128]\n\n");
                    usage(stderr, argc, argv);
                    exit(EXIT_FAILURE);
                }
                break;

            case 'a':
                ave_thresh = atof(optarg);
                break;

            case 'b':
                bright_thresh = atof(optarg);
                break;

            case 'c':
                col_thresh = atof(optarg);
                break;

            default:
                usage(stderr, argc, argv);
                exit(EXIT_FAILURE);
        }
    }

    last_frame.width = capture_width;
    last_frame.height = capture_height;
    last_frame.start = 0;
    last_frame.length = 0;

    struct screen_buf rgb_frame;
    rgb_frame.start = 0;
    rgb_frame.length = 0;

    open_device();
    init_device();
    init_buf();
    start_capturing();
    clock_gettime( CLOCK_REALTIME, &begin);
    while (1) {
        mainloop();
        clock_gettime( CLOCK_REALTIME, &end);
        if ((end.tv_sec - begin.tv_sec) + ((double)(end.tv_nsec - begin.tv_nsec) / (double)BILLION) > 0.1) {
            clock_gettime( CLOCK_REALTIME, &begin);
            //fprintf(stderr, ".\n");
            //YUV422toRGB888(capture_width, capture_height, last_frame.start, rgb_buf);
            get_rgb(&rgb_frame);
            
            //update_movment(rgb_buf);
            update_movment(rgb_frame.start);
            display_image(movment_buf);
            write_JPEG_file("peep_webcam.jpeg", rgb_frame.start, rgb_frame.width, rgb_frame.height, 3);
            
            //float_buf_to_char_buf(average_buf, average_char_buf, capture_width / scale, capture_height / scale, 3);
            //write_JPEG_file("peep_average.jpeg", average_char_buf, capture_width / scale, capture_height / scale, 3);
            
            //write_JPEG_file("peep_movment.jpeg", movment_buf, capture_width / scale, capture_height / scale, 1);

            first_run = 0;
        }
    }
    stop_capturing();
    uninit_buf();
    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}
