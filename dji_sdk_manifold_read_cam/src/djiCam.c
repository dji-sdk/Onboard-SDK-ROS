/* Copyright (C) 2015, DJI Innovations, Inc. All rights reserved.
 *
 * The software is licensed under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License.
 * If not, see <http://www.gnu.org/licenses/>.
 */
 

#include <stdio.h>
#include <jpeglib.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <strings.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <signal.h>

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/ioctl.h>
#include <assert.h>

#include <poll.h>
#include <stdbool.h>
#include <pthread.h>
#include <semaphore.h>

#include "libusb-gadget/src/config.h"
#include "usbGadget.h"
#include "usbHost.h"
#include "djiCam.h"


#define __STDC_CONSTANT_MACROS
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <SDL/SDL.h>



#define SFM_REFRESH_EVENT       (SDL_USEREVENT + 1)
#define BUFFER_SIZE 			512	    /*Max size of usb read/write*/
#define BUF_NUM                 64
#define SCREEN_W                1280    /*720P*/
#define SCREEN_H                720
#define FRAME_SIZE              SCREEN_W*SCREEN_H*3

#define RGB_FORMAT


/*Thread*/
static SDL_Thread *video_tid; 
static pthread_t vedioDisplay_thread;
static pthread_t usbRead_thread;
static pthread_t usbWrite_thread;
static pthread_t saveJPG_thread;


/*FFmpeg*/
static AVCodec *g_pCodec;
static AVCodecContext *g_pContext = NULL;
static AVCodecParserContext *g_pCodecPaser = NULL;
static AVFrame *g_pFrame,*g_pFrameYUV, *g_pFrameRGB;
static struct SwsContext *g_pConvert_ctx;

/*usb*/
static struct usb_gadget_endpoint *g_gadget_endpoint_in, *g_gadget_endpoint_out; /*Use for usb gadget*/
extern int g_hostGet_endpoint_out, g_hostGet_endpoint_in; /*Use for usb host*/
IplImage * g_pImage;

static struct SwsContext* swscontext;
static int linesize[4] = {0, 0, 0, 0};

static volatile int g_trigger = 0;
static int g_gotFrame = 0;
static volatile int g_ForceBreak = 0;
static volatile unsigned int g_nFrame = 0;
static volatile int g_thread_exit = 0;

static pthread_mutex_t frame_mutex;
static pthread_mutex_t hImgMutex;
static int g_mode = 0;

typedef struct framebuf{
    uint8_t data[BUF_NUM][FRAME_SIZE];
    volatile unsigned int nframe[BUF_NUM];
    volatile uint8_t newest[BUF_NUM]; 
    volatile unsigned int r;
    volatile unsigned int w;  
}framebuf_t;
static framebuf_t g_FrameBuf;

typedef struct databuf{
    uint8_t buf[BUF_NUM][BUFFER_SIZE + 8];
    volatile unsigned int len[BUF_NUM];
    volatile unsigned int r;
    volatile unsigned int w;
}databuf_t;
static databuf_t g_DataBuf;

static struct usb_gadget_device device = {
		.device = &loopback_device_descriptor,
		.config = loopback_config,
		.hs_config = loopback_hs_config,
		.strings = &loopback_strings,
	};
static usb_gadget_dev_handle *handle;
static struct pollfd fds;




int av2ipl(AVFrame *src, IplImage *dst, int height, int width)
{
	
    linesize[0] = dst->widthStep;
    if (swscontext == 0)
        return 0;
    sws_scale(swscontext, src->data, src->linesize, 0, height, (uint8_t **) & (dst->imageData), linesize);
    return 1;
}



static int sfp_refresh_thread(void *opaque)
{
	while (0 == g_thread_exit) 
	{
		SDL_Event event;
		event.type = SFM_REFRESH_EVENT;
		SDL_PushEvent(&event);
		SDL_Delay(30);
	}
	printf("sfp_refresh_thread quit ok!\n");
	return 0;
}


static void loopback_stop_endpoints (void *data)
{
    usb_gadget_endpoint_close (g_gadget_endpoint_in);
    usb_gadget_endpoint_close (g_gadget_endpoint_out);
    g_gadget_endpoint_in = NULL;
    g_gadget_endpoint_out = NULL;
}


/*Read data from camera*/
static void *usbRead_loop(void *data)
{
    int w;
        int ret;
        static int count = 0;
        int paserLength_In;
        int paserLen;
        uint8_t *pFrameBuff;
        int i;
        static int found = 0;
        int read_counter = 0;
        AVFrame *pFrame;
        IplImage * pImg = cvCreateImage(cvSize(SCREEN_W,SCREEN_H),IPL_DEPTH_8U,3);

        printf("usbRead_loop thread start ...\n");
        pthread_cleanup_push (loopback_stop_endpoints, NULL);
        sleep(1);
        while(1)
        {

            pthread_testcancel ();

            if(1 == g_ForceBreak)
            {
                printf("Force usbRead_loop thread to stop!\n");
                break;
            }
            w = g_DataBuf.w;
            ret = usb_gadget_endpoint_read(g_gadget_endpoint_out, g_DataBuf.buf[w], BUFFER_SIZE, 100);
            pthread_testcancel ();
            //printf("ret = %d\n",ret);
            if(ret != 37 && read_counter == 0)
            {

                printf("not 37 .................................\n");
            }
            read_counter++;
            if(!(read_counter % 1000))
            {
                printf("*Read %d\n", read_counter);
            }
            if (ret <= 0 || ret > BUFFER_SIZE)
            {
                printf("usbRead_loop:usb_gadget_endpoint_read, ret = %d\n", ret);
                perror ("usbRead_loop:usb_gadget_endpoint_read:");
                continue;
            }
            if(found == 0)
            {
                for(i=0;i<=(ret-5);i++)
                {
                    if(g_DataBuf.buf[w][i]==0x00 && g_DataBuf.buf[w][i+1]==0x00 && g_DataBuf.buf[w][i+2]==0x00 && g_DataBuf.buf[w][i+3]==0x01 && g_DataBuf.buf[w][i+4]==0x67)
                    {
                        found = 1;
                        printf("Found  header..........\n");
                        break;
                    }
                }
                if(found == 0)
                    continue;
            }

            paserLength_In = ret;
            pFrameBuff = g_DataBuf.buf[w];
            g_DataBuf.len[w] = ret;
            g_DataBuf.w = (w + 1) % (BUF_NUM);

            if(!g_trigger)  ++count;
            if(2 == count)
            {
                count = 0;
                g_trigger = 1;
            }

            while(paserLength_In)
            {
                AVPacket avpkt;
                av_init_packet(&avpkt);
                paserLen = av_parser_parse2(g_pCodecPaser, g_pContext, &avpkt.data, &avpkt.size, pFrameBuff, paserLength_In, AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
                paserLength_In -= paserLen;
                pFrameBuff += paserLen;

                if(avpkt.size > 0)
                {
                    pthread_mutex_lock(&frame_mutex);
                    ret = avcodec_decode_video2(g_pContext, g_pFrame, &g_gotFrame, &avpkt);
                    if(ret < 0)
                    {
                        printf("usbRead_loop: decode error ! \n");
                        pthread_mutex_unlock(&frame_mutex);
                        break;
                    }
                    else if(g_gotFrame)
                    {
                        av2ipl(g_pFrame,g_pImage,SCREEN_H,SCREEN_W);
                        g_nFrame++;

                        if(!(g_nFrame % 100))
                        {
                            printf("*Frame %d\n", g_nFrame);
                        }
                    }
                    pthread_mutex_unlock(&frame_mutex);

                }
                av_free_packet(&avpkt);
            }

        }
     pthread_cleanup_pop (1);
}


static void loopback_event_cb (usb_gadget_dev_handle *handle, struct usb_gadget_event *event, void *arg)
{

    switch (event->type)
    {
        case USG_EVENT_ENDPOINT_ENABLE:
            if (event->u.number == (loopback_ep_in_descriptor.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK))
            {
                g_gadget_endpoint_in = usb_gadget_endpoint (handle, event->u.number);
                printf("open ep_in\n");
            }
            else if (event->u.number == (loopback_ep_out_descriptor.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK))
            {
                g_gadget_endpoint_out = usb_gadget_endpoint (handle, event->u.number);
                printf("open ep_out\n");
                if (pthread_create (&usbRead_thread, 0, usbRead_loop, NULL) != 0)
                {
                    perror ("usbRead_thread create");
                    assert(0);
                }
            }

            if (!g_gadget_endpoint_in || !g_gadget_endpoint_out)
                return;
            printf("connect\n");
            break;

        case USG_EVENT_ENDPOINT_DISABLE:
            if (event->u.number == (loopback_ep_in_descriptor.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK))
                g_gadget_endpoint_in = NULL;
            else if (event->u.number == (loopback_ep_out_descriptor.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK))
                g_gadget_endpoint_out = NULL;
            break;

        case USG_EVENT_DISCONNECT:	/* FALLTHROUGH */
            printf("disconnect\n");
            if (usbRead_thread)
                pthread_cancel (usbRead_thread);
            break;

        default:
            break;
    }
}
 
static void decode_init()
{
    memset((void *)&g_FrameBuf,0,sizeof(g_FrameBuf));
    avcodec_register_all();
    g_pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!g_pCodec) {
        fprintf(stderr, "Codec not found\n");
        exit(1);
    }
    g_pContext = avcodec_alloc_context3(g_pCodec);
    if (!g_pContext) {
        fprintf(stderr, "Could not allocate video codec context\n");
        exit(1);
    }
    if(g_pCodec->capabilities & CODEC_CAP_TRUNCATED)
        g_pContext->flags |= CODEC_FLAG_TRUNCATED; /* we do not send complete frames */

    if (avcodec_open2(g_pContext, g_pCodec, NULL) < 0)
    { 	/* open it */
        fprintf(stderr, "Could not open codec\n");
        exit(1);
    }
    g_pFrame = av_frame_alloc();
    if (!g_pFrame) {
        fprintf(stderr, "Could not allocate video g_pFrame\n");
        exit(1);
    }
    g_pFrameYUV=av_frame_alloc();
    if (!g_pFrameYUV) {
        fprintf(stderr, "Could not allocate video g_pFrameYUV\n");
        exit(1);
    }
    g_pFrameRGB=av_frame_alloc();
    if (!g_pFrameRGB) {
        fprintf(stderr, "Could not allocate video g_pFrameRGB\n");
        exit(1);
    }
    g_pCodecPaser = av_parser_init(AV_CODEC_ID_H264);
    if (!g_pCodecPaser)
    {
        fprintf(stderr, "g_pCodecPaser NULL\n");
        exit(1);
    }

}

int djiCam_loop(IplImage* pImg)
{
	int ret;
	fds.revents = 0;
	ret = poll (&fds, 1, 10); 

	if (ret < 0 || g_ForceBreak == 1)
	{
		perror ("poll error or break!\n");
		ret = system("echo disconnect > /sys/devices/platform/tegra-udc.0/udc/tegra-udc.0/soft_connect");
		if(ret < 0)
			printf("system called failed!  %s %d\n",__FILE__,__LINE__);
		return 0;
	}

	if (fds.revents & POLLIN)
		usb_gadget_handle_control_event (handle); 
	if (g_pImage) {
		pthread_mutex_lock(&hImgMutex);
		//cvResize(g_pImage,pImg,0); //use this one for a lower resolution pImg
		cvCopy(g_pImage, pImg, 0); 
		pthread_mutex_unlock(&hImgMutex);
	}

	return 1;
}

static void stopSignalHanle(int sig)
{
    printf("Stop SIGNAL! \n");
    g_ForceBreak = 1;

}
 
int djiCam_init()
{
    int ret;
	int debug_level = 0;
	ret = system("echo 0 > /sys/class/gpio/gpio243/value");
	if(ret < 0)
	{
		printf("system called failed!  %s %d\n",__FILE__,__LINE__);
		//	return -1;
	}
	ret = system("echo 0 > /sys/devices/platform/tegra-otg/enable_host");
	if(ret < 0)
	{
		printf("system called failed!  %s %d\n",__FILE__,__LINE__);
		//	return -1;
	}
	printf("disable host\n");
	ret = system("echo 1 > /sys/devices/platform/tegra-otg/enable_device");
	if(ret < 0)
	{
		printf("system called failed!  %s %d\n",__FILE__,__LINE__);
		//	return -1;
	}
	printf("enable device\n");
	//sleep(1);
	ret = system("mkdir /dev/gadget");
	if(ret < 0)
	{
		printf("system called failed!  %s %d\n",__FILE__,__LINE__);
		//  return -1;
	}
	ret = system("mount -t gadgetfs none /dev/gadget");
	if(ret < 0)
	{
		printf("system called failed!  %s %d\n",__FILE__,__LINE__);
		//	return -1;
	}
	printf("mount gadgetfs\n");

	loopback_device_descriptor.idVendor = OTG_VENDORID;
	loopback_device_descriptor.idProduct = OTG_PRODUCTID;
	handle = usb_gadget_open (&device);
	if (!handle)
	{
		fprintf (stderr, "Couldn't open device.\n");
		return -1;
	}

	usb_gadget_set_event_cb (handle, loopback_event_cb, NULL);
	usb_gadget_set_debug_level (handle, debug_level);

	usb_gadget_endpoint (handle, 0); 


	fds.fd = usb_gadget_control_fd (handle); 
	fds.events = POLLIN | POLLPRI;


	ret = pthread_mutex_init(&frame_mutex, NULL);
    pthread_mutex_init(&hImgMutex, NULL);
	if(ret != 0)
	{
		perror("Mutex init failed");
		return -1;
	}

	signal(SIGINT, stopSignalHanle);
	decode_init();
	video_tid = SDL_CreateThread(sfp_refresh_thread,NULL);

	printf("Init is ok..........\n");
	
	
	ret = system("echo connect > /sys/devices/platform/tegra-udc.0/udc/tegra-udc.0/soft_connect");  
	if(ret < 0)
	{
		printf("system called failed!  %s %d\n",__FILE__,__LINE__);
	}
	printf("udc soft connect\n");	

	g_pImage = cvCreateImage(cvSize(SCREEN_W, SCREEN_H),IPL_DEPTH_8U,3);
	swscontext = sws_getContext(SCREEN_W, SCREEN_H, PIX_FMT_YUV420P, SCREEN_W, SCREEN_H, PIX_FMT_BGR24, SWS_BILINEAR, 0, 0, 0);

	return 0;
}

void djiCam_exit()
{
	void *result = NULL;
	int ret;

	usb_gadget_close(handle);
	ret = system("umount -f /dev/gadget");
	if(ret < 0)
	{
		printf("system called failed!  %s %d\n",__FILE__,__LINE__);
	}
	ret = system("rm -rf /dev/gadget");
	if(ret < 0)
	{
		printf("system called failed!  %s %d\n",__FILE__,__LINE__);
	}

#if 0
	if(usbRead_thread)
		pthread_join(usbRead_thread,&result);
	if(usbWrite_thread)
		pthread_join(usbWrite_thread,&result);
	if(vedioDisplay_thread)
		pthread_join(vedioDisplay_thread,&result);
	if(saveJPG_thread)
		pthread_join(saveJPG_thread,&result);
#endif
	system("echo disconnect > /sys/devices/platform/tegra-udc.0/udc/tegra-udc.0/soft_connect");
	usleep(200*1000);
	system("echo 0 > /sys/devices/platform/tegra-otg/enable_device");
	usleep(200*1000);
	system("echo 1 > /sys/devices/platform/tegra-otg/enable_host");
	usleep(200*1000);
	system("echo 1 > /sys/class/gpio/gpio243/value");
	printf("The threads exit sucessfully !\n");
	usleep(200*1000);

    pthread_mutex_destroy(&frame_mutex);
    pthread_mutex_destroy(&hImgMutex);
	sws_freeContext(g_pConvert_ctx);		
	avcodec_close(g_pContext);
	av_free(g_pContext);
	av_frame_free(&g_pFrame);
	av_frame_free(&g_pFrameYUV);
	av_frame_free(&g_pFrameRGB);

}
