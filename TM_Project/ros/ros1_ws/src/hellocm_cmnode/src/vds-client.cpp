/*
******************************************************************************
**  CarMaker - Version 9.0
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Video Data Stream example client for IPGMovie 3.4 and later versions.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <pthread.h>
#include <semaphore.h>
#endif

#include <Log.h>
#include <DataDict.h>
#include <SimCore.h>
#include <InfoUtils.h>

#include "vds-client.h"
#define LOG(frmt, ...)  Log(frmt "\n", ##__VA_ARGS__)

static struct
{
    char *MovieHost; /* pc on which IPGMovie runs          */
    int MoviePort;   /* TCP/IP port for VDS                */
    int sock;        /* TCP/IP Socket                      */
    char sbuf[64];   /* Buffer for transmitted information */
    int RecvFlags;   /* Receive Flags                      */
    int Verbose;     /* Logging Output                     */
} VDScfg;

struct
{
    int nImages;
    int nBytes;
    float MinDepth;
} VDSIF;

#ifdef WIN32
HANDLE vdsc_thread;
HANDLE vdsc_sem;
#else
pthread_t vdsc_thread;
sem_t vdsc_sem;
#endif

static volatile int vdsc_terminate;
static fd_set vdsc_readfds;

static void *VDS_Thread_Func(void *arg);
std::vector<ros::Publisher> vds_Pub;

static int
VDS_ReadChunk(char *buf, int count)
{
    int nread = 0, res;
    while (nread < count)
    {
        if (vdsc_terminate)
            return -1;

        FD_SET(VDScfg.sock, &vdsc_readfds);

        const int timeout_ms = 100;
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        res = select(VDScfg.sock + 1, &vdsc_readfds, NULL, NULL, &tv);
        if (res == 0)
        {
            continue;
        }
        else if (res < 0)
        {
            //Log("VDS: socket read error: select() failed\n");
            return -1;
        }

        res = recv(VDScfg.sock, buf + nread, count - nread, VDScfg.RecvFlags);
        if (res <= 0)
        {
            //Log("VDS: socket read error: recv() failed\n");
            return -1;
        }
        nread += res;
    }
    return 0;
}

/*
** VDS_RecvHdr
**
** Scan TCP/IP Socket and writes to buffer
*/
static int
VDS_RecvHdr(void)
{
    const int HdrSize = 64;
    char *hdr = VDScfg.sbuf;
    int nSkipped = 0, len = 0, i;

    while (1)
    {
        if (VDS_ReadChunk(hdr + len, HdrSize - len) != 0)
            return -1;
        len = HdrSize;

        if (hdr[0] == '*' && hdr[1] >= 'A' && hdr[1] <= 'Z')
        {
            /* remove whitespace at end of line */
            while (len > 0 && hdr[len - 1] <= ' ')
                len--;
            hdr[len] = '\0';
            if (VDScfg.Verbose && nSkipped > 0)
                Log("VDS: HDR resync, %d bytes skipped\n", nSkipped);
            return 0;
        }

        for (i = 1; i < len && hdr[i] != '*'; i++)
            ;
        len -= i;
        nSkipped += i;
        memmove(hdr, hdr + i, len);
    }
}

/*
** VDS_Connect
**
** Connect over TCP/IP socket
*/
static int
VDS_Connect(void)
{
    struct sockaddr_in DestAddr;
    struct hostent *he;

    if ((he = gethostbyname(VDScfg.MovieHost)) == NULL)
    {
        Log("VDS: unknown host: %s\n", VDScfg.MovieHost);
        return -2;
    }
    DestAddr.sin_family = AF_INET;
    DestAddr.sin_port = htons((unsigned short)VDScfg.MoviePort);
    DestAddr.sin_addr.s_addr = *(unsigned *)he->h_addr;
    VDScfg.sock = socket(AF_INET, SOCK_STREAM, 0);
    if (connect(VDScfg.sock, (struct sockaddr *)&DestAddr, sizeof(DestAddr)) < 0)
    {
        Log("VDS: can't connect '%s:%d'\n", VDScfg.MovieHost, VDScfg.MoviePort);
        return -2;
    }
    if (VDS_RecvHdr() < 0)
        return -3;

    Log("VDS: connected: %s\n", VDScfg.sbuf + 1);

    return 0;
}

/*
** VDS_GetData
**
** data and image processing
*/
static int
VDS_GetData(std::vector<ros::Publisher> Pub)
{
    /* Variables for Image Processing */
    char ImgType[64];
    int ImgWidth, ImgHeight, Channel;
    float SimTime;
    int ImgLen; // length in char
    int nPixel; // actual length
    int Pixel;
    int MinDepthPixel;
    uint8_t *u_img;

    if (sscanf(VDScfg.sbuf, "*VDS %d %s %f %dx%d %d", &Channel,
               ImgType, &SimTime, &ImgWidth, &ImgHeight, &ImgLen) == 6)
    {
        if (0)
            LOG("%6.3f %d: %8s %dx%d %d\n", SimTime, Channel, ImgType, ImgWidth, ImgHeight, ImgLen);
        if (ImgLen > 0)
        {
            char *img = (char *)malloc(ImgLen);
            if (VDS_ReadChunk(img, ImgLen) != 0)
            {
                free(img);
                return -1;
            }
            /* USER BEGIN: Publish frames to ROS */
            if (strcmp(ImgType, "rgb") == 0)
            {
                nPixel = ImgLen/sizeof(uint8_t);
                u_img = (uint8_t *)img;
                sensor_msgs::Image msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "/camera";
                msg.data = std::vector<uint8_t>(u_img, u_img + ImgLen);
                msg.height = ImgHeight;
                msg.width = ImgWidth;
                msg.encoding = "rgb8";
                msg.step = 3 * ImgWidth;
                msg.is_bigendian = false;
                Pub[Channel].publish(msg);
            }
            if (strcmp(ImgType, "depth") == 0)
            {
                nPixel = ImgLen/sizeof(uint8_t);
                u_img = (uint8_t *)img;
                sensor_msgs::Image msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "/camera";
                msg.data = std::vector<uint8_t>(u_img, u_img + ImgLen);
                msg.height = ImgHeight;
                msg.width = ImgWidth;
                msg.encoding = "32FC1";
                msg.step = 4 * ImgWidth;
                msg.is_bigendian = false;
                Pub[Channel].publish(msg);
            }
            /* USER END: Publish frames to ROS */
            free(img);
        }
        VDSIF.nImages++;
        VDSIF.nBytes += ImgLen;
    }
    else
    {
        Log("VDS: not handled: %s\n", VDScfg.sbuf);
    }

    return 0;
}

/*
** VDS_Init
**
** Initialize Data Struct
** Create VDS Thread
** Define DataDict Variables
*/
void VDS_Init(std::vector<ros::Publisher> Pub_in)
{
    int i;
    /* ROS */
    vds_Pub = Pub_in;

    VDScfg.MovieHost = "localhost";
    VDScfg.MoviePort = 2210;
    VDScfg.Verbose = 0;
    VDScfg.RecvFlags = 0;
    memset(VDScfg.sbuf, 0, 64);
    vdsc_terminate = 0;

    VDSIF.nImages = 0;
    VDSIF.nBytes = 0;
    VDSIF.MinDepth = 0;

#ifdef WIN32
    /* Initialize Semaphore for Thread Synchronisation */
    vdsc_sem = CreateSemaphore(NULL, 0, 1, NULL);

    /* Create Thread for Data Aquisition and Processing */
    vdsc_thread = CreateThread(NULL, 100 * 1024,
                               (LPTHREAD_START_ROUTINE)VDS_Thread_Func, NULL, 0, NULL);
    i = vdsc_thread == NULL ? -1 : 0;
#else
    /* Initialize Semaphore for Thread Synchronisation */
    sem_init(&vdsc_sem, 0, 1);

    /* Create Thread for Data Aquisition and Processing */
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 100 * 1024);
    vdsc_thread = -1;
    i = pthread_create(&vdsc_thread, &attr, VDS_Thread_Func, &vds_Pub);
    pthread_attr_destroy(&attr);
#endif

    if (i != 0)
    {
        LogErrF(EC_General, "VDS: unable to create background thread (returns %d, %s)\n",
                i, strerror(errno));
    }

    /* Define UAQ's to be used as Interface to CarMaker */
    DDefFloat(NULL, "VDS.MinDepth", "m", &VDSIF.MinDepth, DVA_None);
    DDefInt(NULL, "VDS.nImages", "", &VDSIF.nImages, DVA_None);
    DDefInt(NULL, "VDS.nBytes", "", &VDSIF.nBytes, DVA_None);
}

/*
** VDS_Exit
**
** Signal termination request to background thread,
** wait for it to complete, then clean up.
*/
void VDS_Exit(void)
{
    vdsc_terminate = 1;

#ifdef WIN32
    ReleaseSemaphore(vdsc_sem, 1, NULL);
    if (vdsc_thread != NULL)
        WaitForSingleObject(vdsc_thread, INFINITE);

    CloseHandle(&vdsc_sem);
#else
    sem_post(&vdsc_sem);
    if (vdsc_thread != -1)
        pthread_join(vdsc_thread, NULL);

    sem_destroy(&vdsc_sem);
#endif
}

/*
** VDS_Start
**
** Restart VDS Thread if not already started
*/
void VDS_Start(void)
{
#ifdef WIN32
    ReleaseSemaphore(vdsc_sem, 1, NULL);
#else
    int i;
    sem_getvalue(&vdsc_sem, &i);
    if (i == 0)
        sem_post(&vdsc_sem);
#endif

    VDScfg.MovieHost = iGetStrOpt(SimCore.TestRig.SimParam.Inf, "VDS.MovieHost", "localhost");
    VDScfg.MoviePort = iGetDblOpt(SimCore.TestRig.SimParam.Inf, "VDS.MoviePort", 2210);
    VDScfg.Verbose = iGetDblOpt(SimCore.TestRig.SimParam.Inf, "VDS.Verbose", 0);
}

/*
** Read_VDS
**
** VDS Thread
*/
static void *
VDS_Thread_Func(void *arg)
{
    int i;
    std::vector<ros::Publisher> Pub = *((std::vector<ros::Publisher> *) arg);

    /* Wait (blocking) for semaphore */
#ifdef WIN32
    while (!vdsc_terminate && WaitForSingleObject(vdsc_sem, INFINITE) != WAIT_FAILED)
    {
#else
    int j;
    sem_getvalue(&vdsc_sem, &j);
    LOG("termination: %d, sem: %d", vdsc_terminate, j);
    while (!vdsc_terminate && sem_wait(&vdsc_sem) == 0)
    {
#endif
        if (vdsc_terminate)
            break;

        /* Connect to VDS Server */
        if ((i = VDS_Connect()) != 0)
        {
            LogWarnF(EC_General, "VDS: unable to connect (returns %d, %s)\n",
                     i, strerror(errno));
            continue;
        }

        /* Read from TCP/IP-Port and process the image */
        while (1)
        {
            if (VDS_RecvHdr() != 0)
                break;
            if (VDS_GetData(Pub) != 0)
                break;
        }

#ifdef WIN32
        closesocket(VDScfg.sock);
#else
        close(VDScfg.sock);
#endif
    }

    return NULL;
}
