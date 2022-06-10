/*
******************************************************************************
**  CarMaker - Version 10.0
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Raw Signal Data Stream example for IPGMovie 3.4 and later versions.
**
** Compiling:
** Linux
**	gcc -Wall -Os -o rsds-client rsds.c
** MS Windows (MSYS/MinGW)
**	gcc -Wall -Os -o rsds-client.exe rsds.c -lws2_32
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#ifdef WIN32
#  include <winsock2.h>
#else
#  include <unistd.h>
#  include <sys/socket.h>
#  include <sys/types.h>
#  include <net/if.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <netdb.h>
#endif

// CarMaker Includes
#include "rsds-client.h"
#include "InfoUtils.h"
#include "infoc.h"
#include "infofile.h"

// ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/TwistStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ros/package.h"

#include <angles/angles.h>
#include <vector>
#include <iostream>

#define FreeIfNotNULL(a) if ((a)!=NULL) do {free (a); (a)=NULL;} while(0)

int nSens_Radar_Last = 0;

// Variables from Sensor InfoFile, global def
double* nBeams;
double* BeamTable;
int rows = 0;
int point_step, row_step;

// Global shared pointer 
sensor_msgs::PointCloud2Ptr pointcloud = boost::make_shared<sensor_msgs::PointCloud2>();

// Misc. variables used in pointcloud msg packaging
float x, y, z; // 3D Coordinate system points 
double distance, azimuth, elevation;

/*
** RSDS_RecvHdr
**
** Scan TCP/IP Socket and writes to buffer
*/
static int
RSDS_RecvHdr (int sock, char *hdr)
{
    const int HdrSize = 64;
    int len = 0;
    int nSkipped = 0;
    int i;

    while (1) {
        for ( ; len < HdrSize; len += i) {
	    if ((i=recv(sock, hdr+len, HdrSize-len, RSDScfg.RecvFlags)) <= 0)
		return -1;
	}
	if (hdr[0]=='*' && hdr[1]>='A' && hdr[1]<='Z') {
	    /* remove white spaces at end of line */
	    while (len>0 && hdr[len-1]<=' ')
		len--;
	    hdr[len] = 0;
	    if (RSDScfg.Verbose && nSkipped>0)
	        printf ("RSDS: HDR resync, %d bytes skipped\n", nSkipped);
	    return 0;
	}
	for (i=1; i<len && hdr[i]!='*'; i++) ;
	len -= i;
	nSkipped += i;
	memmove (hdr, hdr+i, len);
    }
}



/*
** RSDS_Connect
**
** Connect over TCP/IP socket
*/
static int
RSDS_Connect (void)
{
#ifdef WIN32
    WSADATA WSAdata;
    if (WSAStartup(MAKEWORD(2,0), &WSAdata) != 0) {
        fprintf (stderr, "RSDS: WSAStartup ((2,0),0) => %d\n", WSAGetLastError());
        return -1;
    }
#endif

    struct sockaddr_in	DestAddr;
    struct hostent  	*he;

    int tries = RSDScfg.ConnectionTries;

    if ((he=gethostbyname(RSDScfg.MovieHost.c_str())) == NULL) {
        fprintf (stderr, "RSDS: unknown host: %s\n", RSDScfg.MovieHost.c_str());
        return -2;
    }
    DestAddr.sin_family      = AF_INET;
    DestAddr.sin_port        = htons((unsigned short)RSDScfg.MoviePort);
    DestAddr.sin_addr.s_addr = *(unsigned *)he->h_addr;
    RSDScfg.sock = socket(AF_INET, SOCK_STREAM, 0);

    printf("Hostip : %s \nHostport : %d \nSocket : %d \n", RSDScfg.MovieHost.c_str(), RSDScfg.MoviePort, RSDScfg.sock);

    fflush(stdout);

    while (connect(RSDScfg.sock, (struct sockaddr *) &DestAddr, sizeof(DestAddr)) < 0 && tries > 0){
	fprintf(stderr, "RSDS: can't connect '%s:%d'\n", RSDScfg.MovieHost.c_str(), RSDScfg.MoviePort);
	if (tries > 1) {
            fprintf(stderr, "\tretrying in 1 second... (%d)\n", --tries);
            fflush(stderr);
            sleep(1);
        } else {
            return -4;
        }
    }

    if (RSDS_RecvHdr(RSDScfg.sock, RSDScfg.hbuf) < 0)
        return -3;

    printf ("RSDS: Connected: %s\n", RSDScfg.hbuf+1);

    return 0;
}


/*
** RSDS_GetData
**
** data and image processing
*/
static int
RSDS_GetData(void)
{
    int len = 0;
    int res = 0;

    /* Variables for Image Processing */
    char    SensorType[64];
    float   SimTime;
    int     DataLen;

    if (sscanf(RSDScfg.hbuf, "*%s %f %d", SensorType, &SimTime, &DataLen) == 3) {
        if (RSDScfg.Verbose)
            printf ("#%6d -- %8s -> t = %6.3f -- s = %d\n",RSDSIF.nFrames, SensorType, SimTime, DataLen);
        if (DataLen > 0) {
            char *data = (char *)malloc(DataLen);
            for (len=0; len<DataLen; len+=res) {
                if ((res=recv(RSDScfg.sock, data+len, DataLen-len, RSDScfg.RecvFlags)) < 0) {
                    printf("RSDS: Socket Reading Failure\n");
                    break;
                }
            }

            if (len == DataLen) {		
				if (strcmp(SensorType, "LidarRSI") == 0) {
					processLidarData(data);
				}
            }

            FreeIfNotNULL(data);
        }
        RSDSIF.nFrames++;
        RSDSIF.nBytes += len;
    } else {
        printf ("RSDS: not handled: %s\n",RSDScfg.hbuf);
    }

    return 0;
}


void processLidarData(const char *dataP)
{
    tRSIResMsg *ResMsg = (tRSIResMsg*) dataP;
    // ATTENTION: nSensors only corresponds to the sensors running on the GPUSensor
    // which is sending the data refer to SensorID in tRSIResMsg_Header_Lidar
    // for the global sensor ID 
    int nSensors = ResMsg->Header.nSensors;

    tScanPoint_L *ScanPoints[nSensors];
    int nScanPoints[nSensors];

    tRSIResMsg_Lidar *DataBlockLidar = (tRSIResMsg_Lidar*) (ResMsg->Data);

    // get data of this DataBlock
    nScanPoints[0] = DataBlockLidar->Header.nScanPoints;
    ScanPoints[0] = DataBlockLidar->SP;

    // Header time stamp
    pointcloud->header.stamp = ros::Time::now(); 

    // Unordered cloud with the number of scanned points
    pointcloud->width = nScanPoints[0];

    // Lidar messages are 36 bytes
    point_step = 36;
    row_step = point_step * nScanPoints[0];
    pointcloud->point_step = point_step;
    pointcloud->row_step = row_step;
    
    // Create the binary blob
    uint8_t *ptr; //blob stepping pointer
    pointcloud->data.resize(row_step);
    ptr = pointcloud->data.data();

    for (int i = 0; i < nScanPoints[0]; i++) {
        
        // Spherical to cartesian conversion
        distance    = DataBlockLidar->SP[i].LengthOF / 2; //Distance is half of total length of flight
        azimuth     = angles::from_degrees(BeamTable[4 * rows + DataBlockLidar->SP[i].BeamID]); // BeamID is the table row, and Azimuth is column 5
        elevation   = angles::from_degrees(BeamTable[5 * rows + DataBlockLidar->SP[i].BeamID]); // BeamID is the table row, and Elevation is column 6
        x           = distance * cos(elevation) * cos(azimuth);
        y           = distance * cos(elevation) * sin(azimuth);
        z           = distance * sin(elevation);

        // Package current fields into msg
        *(reinterpret_cast<uint32_t*>(ptr + 0)) = DataBlockLidar->SP[i].BeamID;
        *(reinterpret_cast<double*>(ptr + 4))   = DataBlockLidar->SP[i].TimeOF;
        *(reinterpret_cast<double*>(ptr + 12))  = DataBlockLidar->SP[i].LengthOF;
        *(reinterpret_cast<float*>(ptr + 20))   = x;
        *(reinterpret_cast<float*>(ptr + 24))   = y;
        *(reinterpret_cast<float*>(ptr + 28))   = z;
        *(reinterpret_cast<float*>(ptr + 32))   = DataBlockLidar->SP[i].Intensity;
        
        // Increment offsets
        ptr += point_step;
    }

    // jump to next memory address after current data block to get correct new DataBlock position
    DataBlockLidar =(tRSIResMsg_Lidar*) &(DataBlockLidar->SP[DataBlockLidar->Header.nScanPoints]);
}

/*
** RSDS_Init
**
** Initialize Data Struct
*/
void
RSDS_Init(void)
{
    // RSDS Parameters
    RSDScfg.MovieHost = "localhost";
    RSDScfg.MoviePort = 2210;
    RSDScfg.Verbose   = 0;
    RSDScfg.RecvFlags = 0;
    RSDScfg.ConnectionTries = 10;

    RSDSIF.nFrames  = 0;
    RSDSIF.nBytes   = 0;

    // Set Fields of pcl2 point cloud
    // Frame ID
    pointcloud->header.frame_id = "Fr1A";
    
    // Beam ID
    pointcloud->fields[0].name = "beam_id";  //name of field
    pointcloud->fields[0].offset = 0;        //start with zero offset
    pointcloud->fields[0].datatype = 5;      //int32
    pointcloud->fields[0].count = 1;         //number of elements in field

    // Time of Flight
    pointcloud->fields[1].name = "time_of";
    pointcloud->fields[1].offset = 4;        //4 byte offset
    pointcloud->fields[1].datatype = 8;      //double (float64)
    pointcloud->fields[1].count = 1;

    // Length of Flight
    pointcloud->fields[2].name = "length_of";
    pointcloud->fields[2].offset = 12;       //12 byte offset int (4 bytes) + double (8 bytes)
    pointcloud->fields[2].datatype = 8;      //double
    pointcloud->fields[2].count = 1;

    // X-Coordinate
    pointcloud->fields[3].name = "x";
    pointcloud->fields[3].offset = 20;       //12 + 8 (double)
    pointcloud->fields[3].datatype = 7;      //float32 (ROS preference is float for x/y/z data)
    pointcloud->fields[3].count = 1;

    // Y-Coordinate
    pointcloud->fields[4].name = "y";
    pointcloud->fields[4].offset = 24;       //20 + 4 (float)
    pointcloud->fields[4].datatype = 7;      //float
    pointcloud->fields[4].count = 1;

    // Z-Coordinate
    pointcloud->fields[5].name = "z";
    pointcloud->fields[5].offset = 28;       //24 + 4
    pointcloud->fields[5].datatype = 7;
    pointcloud->fields[5].count = 1;

    // Intensity
    pointcloud->fields[6].name = "intensity";
    pointcloud->fields[6].offset = 32;       //28 + 4...total size 32 + 4 = 36 bytes 
    pointcloud->fields[6].datatype = 7;      //float
    pointcloud->fields[6].count = 1;

    // Misc.
    pointcloud->height = 1;
    pointcloud->is_bigendian = false;
    pointcloud->is_dense = true;

    // Read Lidar InfoFile Data
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = "Fr1A";
    tf.child_frame_id = "Fr_Lidar";

    tInfos *Inf_Sensor = nullptr;
    tErrorMsg *err = nullptr;

    Inf_Sensor = InfoNew();

    // Find CMNode Path used by CMRosIF (allows running rsds_client_node in different workspace)
    std::string path_sensorParam = ros::package::getPath("hellocm_cmnode");
    
    // Trim CMNode Path to top of project
    int str2trim = 31; // amount of characters to trim
    path_sensorParam.erase(path_sensorParam.size() - str2trim);
    
    // Add path for Sensor Parameter InfoFile
    path_sensorParam = path_sensorParam + "/Data/Sensor/LidarRSI_pandarQT_M"; //change file name when beam file name changes
    
    // Read Sensor InfoFile
    int InfoErr = iRead2(&err, Inf_Sensor, path_sensorParam.c_str(), "LidarCode");

    if (InfoErr == 0) {
        
        nBeams = iGetFixedTable2(Inf_Sensor, "Beams.N", 2, 1);
        BeamTable = iGetTableOpt2(Inf_Sensor, "Beams", NULL, 6, &rows);
        
        double *fov_h = iGetFixedTable2(Inf_Sensor, "Beams.FoVH", 2, 1);
        double *fov_v = iGetFixedTable2(Inf_Sensor, "Beams.FoVV", 2, 1);

        if (RSDScfg.Verbose) {
            endl(std::cout << "FOVh = " << fov_h[0] + fov_h[1] << " FOVv = " << fov_v[0] + fov_v[1] << ", nBeams_h = " << nBeams[0] << ", nBeams_v = " << nBeams[1]);
            endl(std::cout << "Beam Table --> First Azimuth Element = " << BeamTable[4*rows]); 
            endl(std::cout << "Beam Table --> First Elevation Element = " <<  BeamTable[5*rows]); 
        }

    } else {
        ROS_ERROR("Something went wrong reading the InfoFile Data/Sensor/LidarRSI, retVal = %d, err = %s", InfoErr, err->Msg);
    }

    InfoDelete(Inf_Sensor);

}


int
main (int argc, char **argv)
{
	// Initialize ROS, node handle, and publisher
	ros::init(argc, argv, "rsds_lidar_node");
	ros::NodeHandle nh;
    ros::Publisher pub;

    // Create PointCloud
    pointcloud->fields.resize(7);
    
    // Transform broadcaster
    tf2_ros::StaticTransformBroadcaster br;

    // Advertise PCL2 topic
	pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud", 1);

    // Misc. ROS
    ros::Rate loop_rate(25); //input is Hz, match to CM sensor cycle time in ms
    int cycleCount = 0;

    // Translation and Rotation
    std::vector<double> param_trans_rot;
    param_trans_rot.assign(6, 0);
    nh.param("param_trans_rot", param_trans_rot, param_trans_rot);

    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time(0);
    tf.header.frame_id = "Fr1A";
    tf.child_frame_id = pointcloud->header.frame_id;
    tf.transform.translation = tf2::toMsg(tf2::Vector3( param_trans_rot[0], 
                                                        param_trans_rot[1], 
                                                        param_trans_rot[2]));
    
    tf2::Quaternion rot;
    tf2::Vector3 default_offset(-90, 0, -90);
    
    rot.setRPY(angles::from_degrees(param_trans_rot[3] + default_offset[0]),
               angles::from_degrees(param_trans_rot[4] + default_offset[1]),
               angles::from_degrees(param_trans_rot[5] + default_offset[2]));
    
    tf.transform.rotation = tf2::toMsg(rot);
    br.sendTransform(tf);

    // Start RSDS Client
    RSDS_Init();

    // Get TCP port of current namespace
    if (nh.getParam(ros::this_node::getNamespace() + "/tcp_port", RSDScfg.MoviePort) == true) {
        printf("Received ROS Parameter tcp_port: %i\n", RSDScfg.MoviePort);
    } else {
        printf("TCP Port parameter was not read! Please check launch files and sensor configuration!\n");
    };

    /* Connect to RSDS Server */
    int i;
    if ((i = RSDS_Connect ()) != 0) {
	    fprintf(stderr, "Can't initialise RSDS client (returns %d, %s)\n",
		    i, i == -4 ? "No server": strerror(errno));
            return 0;
    }

    while (ros::ok()) {

        /* Read from TCP/IP-Port and process the point data */
        while (RSDS_RecvHdr(RSDScfg.sock,RSDScfg.hbuf) == 0) {
            
            // Get RSDS data from TCP stream
            RSDS_GetData();
        
            // Publish PointCloud
            pub.publish(pointcloud);
            
            // Turn over ROS node
            ros::spinOnce();
            loop_rate.sleep();
            cycleCount++;

        }
    }

    printf("Closing RSDS-Client...\n");
    printf("TOTAL: %d Frames received, for %d bytes (%dMB)\n", 
            RSDSIF.nFrames, RSDSIF.nBytes, RSDSIF.nBytes/1024/1024);
    
    ros::shutdown;

    #ifdef WIN32
        closesocket (RSDScfg.sock);
    #else
        close (RSDScfg.sock);
    #endif

    return 0;
}
