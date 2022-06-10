/*!
 ******************************************************************************
 **  CarMaker - Version 8.0.2
 **  Vehicle Dynamics Simulation Toolkit
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 *
 * Description:
 * - Prototype/Proof of Concept
 * - Unsupported ROS Example with CarMaker
 * - Structure may change in future!
 * - Change general parameters in Infofile for CMRosIF ("Data/Config/CMRosIFParameters")
 * - Basic communication with or without parameterizable synchronization
 */

/*!
* Changes:
* - Add LiDAR message (Jianing Lin, 06/08/2020)
*/

/* CarMaker
 * - include other headers e.g. to access to vehicle data
 *   - e.g. "Vehicle.h" or "Vehicle/Sensor_*.h".
 * - additional headers can be found in "<CMInstallDir>/include/"
 * - see Reference Manual, chapter "User Accessible Quantities" to find some variables
 *   that are already defined in DataDictionary and their corresponding C-Code Name
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <cmath>
#include "std_msgs/String.h"
#include "string"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#include "Log.h"
#include "DataDict.h"
#include "SimCore.h"
#include "InfoUtils.h"

#include "apo.h"
#include "GuiCmd.h"

#include "DirectVarAccess.h"
#include "DrivMan.h"
#include "Vehicle.h"
#include "VehicleControl.h"
#include "Car/Car.h"
#include "Car/Steering.h"
#include "Vehicle/Sensor_Object.h"
#include "Vehicle/VehicleControlApps.h"
#include "Traffic.h"
#include "Environment.h"
#include "road.h"
#include "Vehicle/Sensor_Line.h"
#include "Vehicle/Sensor_LidarRSI.h"
#include "Vehicle/Sensor_Object.h"
#include "Vehicle/Sensor_Inertial.h"

/* 2/2/2021 */
/* Add GNSS output */
#include "Vehicle/Sensor_GNav.h"
#include "std_msgs/Int16.h"
#include "gps_common/GPSFix.h"

#include "std_msgs/Bool.h"  // for brake signal

static const unsigned int tablesize = 600000;

/* ROS */
#include "cmrosutils/CMRosUtils.h"    /* Node independent templates, ...*/
#include "cmrosutils/CMRosIF_Utils.h" /* Only for CarMaker ROS Node!!! Functions are located in library for CarMaker ROS Interface */
#include "cmrosutils/CMRemote.h"      /* Basic service for CarMaker remote from ROS */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <angles/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
/* VDS client for publishing camera */
#include "vds-client.h"

/* function flags */
#define VDS 0

/* Following header from external ROS node can be used to get topic/service/... names
 * Other mechanism:
 * 1. Put names manually independently for each node
 * 2. Using command line arguments or launch files and ROS remapping
 * - Doing so, only general message headers are necessary
 */
#if 1
#include "hellocm/ROS1_HelloCM.h" /* External ROS Node. Topic name, ... */
#else
#include <hellocm_msgs/Ext2CM.h>
#include <hellocm_msgs/CM2Ext.h>
#include <hellocm_msgs/Init.h>
#include <hellocm_msgs/CM2Ext_vhcl.h>
#endif

/*! String and numerical version of this Node
 *  - String:    e.g. <Major>.<Minor>.<Patch>
 *  - Numerical: e.g. <nDigitsMajor><2DigitsMinor><2DigitsPatch>
 */
#define CM_VERSION "10.0"
#define CM_NUMVER 100000

/*! String and numerical version of this Node
 *  - String:    e.g. <Major>.<Minor>.<Patch>
 *  - Numerical: e.g. <nDigitsMajor><2DigitsMinor><2DigitsPatch>
 */
#define CMNODE_VERSION "0.8.0"
#define CMNODE_NUMVER 800

/* NDEBUG is set in CarMaker Makefile/MakeDefs in OPT_CFLAGS */
#if !defined NDEBUG
#warning "Debug options are enabled!"
#define DBLOG LOG
#else
#define DBLOG(...)
#endif

/* Not beautiful but consistent with external ROS Node
 * where ROS_INFO is used (implicit newline)*/
#define LOG(frmt, ...) Log(frmt "\n", ##__VA_ARGS__)



double max_scan = 0;
int flag_laser = 0;

void controlCallback(const std_msgs::String msg)
{
    //string command = msg.data;
}

/* General switches for CarMaker ROS Node */
typedef enum tCMNode_Mode
{
    CMNode_Mode_Disabled = 0, /*!< Node is disabled. e.g. don't publish. */
    CMNode_Mode_Default = 1,  /*!< Node is enabled, spinOnce is used  */
    CMNode_Mode_Threaded = 2  /*!< Node is enabled, spin in parallel thread
                                     - Messages are received all the time
                                     - Data is updated at defined position, e.g. *_In()
                                     - Currently not implemented! */
} tCMNode_Mode;

/* Managing synchronization between CarMaker Node and other ROS nodes */
typedef enum tCMNode_SyncMode
{
    CMNode_SyncMode_Disabled = 0, /*!< No synchronization on CarMaker side */
    CMNode_SyncMode_Tpc = 1       /*!< Buffer messages or Spin until external Topics are received */
} tCMNode_SyncMode;

/* Global struct for this Node */
static struct
{
    unsigned long CycleNoRel; /*!< CarMaker relative cycle number, e.g. since start of TestRun */

    struct
    {
        double Duration;   /*!< Time spent for synchronization task */
        int nCycles;       /*!< Number of cycles in synchronization loop */
        int CyclePrepDone; /*!< Last cycle when preparation was done */
        int CycleJobDone;  /*!< Last cycle when job was done */
        double SynthDelay; /*!< Synthetic delay in seconds provided to external node to check sync */
    } Sync;                /*!< Synchronization related information */

    struct
    {
        int CycleNo; /*!< Cycle number of external ROS Node (only for information) */

        /* For debugging */
        int CycleLastOut;   /*!< Cycle number when Topic was published */
        int CycleLastIn;    /*!< Cycle number when Topic from external ROS Node was received */
        int CycleLastFlush; /*!< Cycle number when data from external ROS Node was provided to model */
    } Model;                /*!< Model related information. ROS side! */

    struct
    {
        struct
        {
            tRosIF_TpcSub<hellocm_msgs::Ext2CM> Ext2CM; /* For this example also used for Synchronization */
            tRosIF_TpcSub<sensor_msgs::NavSatFix> GPSin;
            /* 2/8/2021 */
            tRosIF_TpcSub<std_msgs::Int16> steering_angle;
            /* 2/8/2021 */

            /* 3/11/2021 */
            tRosIF_TpcSub<std_msgs::Bool> brake_signal;
            /* 3/11/2021 */
        } Sub;                                          /*!< Topics to be subscribed */

        struct
        {
            tRosIF_TpcPub<hellocm_msgs::CM2Ext> CM2Ext;

            /* USER BEGIN: add publisher into struct */
            //tRosIF_TpcPub<sensor_msgs::PointCloud2> Lidar;
            //tRosIF_TpcPub<sensor_msgs::PointCloud2> Lidar2;
            tRosIF_TpcPub<sensor_msgs::Image> Image0;
            tRosIF_TpcPub<sensor_msgs::Image> Image1;
            tRosIF_TpcPub<sensor_msgs::Image> Image2;
            tRosIF_TpcPub<sensor_msgs::Image> Image3;
            tRosIF_TpcPub<visualization_msgs::MarkerArray> Object;
            tRosIF_TpcPub<geometry_msgs::PoseStamped> OdomIMU;
            tRosIF_TpcPub<nav_msgs::Path> PathIMU;
            /* 2/2/2021 */
            tRosIF_TpcPub<gps_common::GPSFix> GNSS_out;
            /* 2/2/2021 */
            /* 2/8/2021 */
            tRosIF_TpcPub<hellocm_msgs::CM2Ext_vhcl> vhcl_out;
            /* 2/8/2021 */
            /* USER END: add publisher into struct */

            /*!< CarMaker can be working as ROS Time Server providing simulation time
             *   starting at 0 for each TestRun */
            tRosIF_TpcPub<rosgraph_msgs::Clock> Clock;
            //tRosIF_TpcPub<std_msgs::Header> Clock;
        } Pub; /*!< Topics to be published */
    } Topics;  /*!< ROS Topics used by this Node */

    struct
    {
        /*!< Initialization/Preparation of external ROS Node e.g. when simulation starts */
        tRosIF_Srv<hellocm_msgs::Init> Init;
        tRosIF_Srv<cmrosutils::CMRemote> CMRemote; // Trial
    } Services;                                    /*!< ROS Services used by this Node (client and server)*/

    /* USER BEGIN: add struct */
    /*
    struct
    {
        double FOV_h;
        double FOV_v;
        double nBeams_h;
        double nBeams_v;
        double BeamTable[tablesize];
        int ret, rows = 0;
        double x[tablesize / 6];
        double y[tablesize / 6];
        double z[tablesize / 6];
        double *nBeams;

        double *position;
        struct
        {
            double x;
            double y;
            double z;
        } pos;
        double *rotation;
        struct
        {
            double x;
            double y;
            double z;
        } rot;
        char *mountFrame;
        double cycleTime;
        double cycleOffset;
    } LidarRSI;
    */

    struct
    {
        double *position;
        struct
        {
            double x;
            double y;
            double z;
        } pos;
        double *rotation;
        struct
        {
            double x;
            double y;
            double z;
        } rot;
        char *mountFrame;
        double cycleTime;
        double cycleOffset;

        int errCode;
    } ObjectSensor;

    /* 2/3/2021 */
    struct 
    {
        double *position;
        struct
        {
            double x;
            double y;
            double z;
        } pos;
        double *rotation;
        struct
        {
            double x;
            double y;
            double z;
        } rot;

        char *mountFrame;
        double cycleTime;
        double cycleOffset;
    } GNSS;
    /* 2/3/2021 */
    /* USER END: add struct */

    struct
    {
        int QueuePub;     /*!< Queue size for Publishers */
        int QueueSub;     /*!< Queue size for Subscribers */
        int nCyclesClock; /*!< Number of cycles publishing /clock topic.
                                             CycleTime should be multiple of this value */
        tCMNode_Mode Mode;
        tCMNode_SyncMode SyncMode;
        double SyncTimeMax; /* Maximum Synchronization time */

        tRosIF_Cfg Ros;
    } Cfg; /*!< General configuration for this Node */

} CMNode;

/* 2/3/2021 */
/* Global struct for DBW */
static struct
{
    bool GolableEnable = false;

    double steering;
    double brake=0.0;
    double throttle=0.08;
    int gear;
} DBW;
/* 2/3/2021 */
/*!
 * Description:
 * - Callback for ROS Topic published by external ROS Node
 *
 */
static void
cmnode_HelloCM_CB_TpcIn(const hellocm_msgs::Ext2CM::ConstPtr &msg)
{
    /* Process message only if receive is expected */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
        return;

    int rv;
    auto in = &CMNode.Topics.Sub.Ext2CM;

    /* Update receive buffer
     * - No lock for spinOnce necessary?
     */
    in->Msg.header = msg->header;
    in->Msg.time = msg->time;
    in->Msg.cycleno = msg->cycleno;

    // in->Msg.throttle = msg->throttle;
    // in->Msg.brake = msg->brake;
    // in->Msg.steering = msg->steering;
    // in->Msg.gear = msg->gear;

    DBW.GolableEnable = msg->GolableEnable;

    DBW.steering = msg->steering;
    DBW.throttle = msg->throttle;
    DBW.brake = msg->brake;
    DBW.gear = msg->gear;

    /* Stopping simulation is only necessary when synchronization is activated */
    if (CMNode.Cfg.SyncMode == CMNode_SyncMode_Tpc && (rv = CMCRJob_DoPrep_SetDone(in->Job, CMNode.CycleNoRel)) != CMCRJob_RV_Success)
    {
        LogErrF(EC_Sim, "CMNode: Error on DoPrep_SetDone for Job '%s'! rv=%s", CMCRJob_GetName(in->Job), CMCRJob_RVStr(rv));
    }

    /* Remember cycle for debugging */
    CMNode.Model.CycleLastIn = CMNode.CycleNoRel;

    LOG("%s (CMSimTime=%.3fs): External Node is in cycle %lu, Time=%.3fs, Stamp=%.3fs, SeqID=%d",
        ros::this_node::getName().c_str(), SimCore.Time,
        in->Msg.cycleno, msg->time.toSec(), in->Msg.header.stamp.toSec(), in->Msg.header.seq);
}

/* USER BEGIN: callback for GPS */
/*!
 * Description:
 * - Callback for ROS Topic published by external ROS Node
 *
 */
double init_GPS_pose[2] = {-1, -1};
static void
cmnode_callback_GPSin(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if(init_GPS_pose[0] == -1){
        init_GPS_pose[0] = msg->latitude;
        init_GPS_pose[1] = msg->longitude;
    }
    double y = 500 + 111132.944444444 * (msg->latitude - init_GPS_pose[0]);
    double x = 500 + 111319.391666668 * cos(msg->longitude/180*3.1415926) * (msg->longitude - init_GPS_pose[1]);
    double yaw = 0;
    std::cout << "get GPS data..." << std::endl;
    DVA_WriteRequest("vhcl.poi.tx", OWMode_Abs, 1000, 0, 0, x, NULL);
    DVA_WriteRequest("vhcl.poi.ty", OWMode_Abs, 1000, 0, 0, y, NULL);
}
/* USER END: callback for GPS */

/* 2/8/2021 */
/* USER BEGIN: callback for steering angle */
/*!
 * Description:
 * - Callback for ROS Topic published by external ROS Node
 *
 */
static void
cmnode_callback_steering(const std_msgs::Int16::ConstPtr &msg)
{
    DBW.steering = DEG2RAD(msg->data);
}
/* USER END: callback for steering angle */
/* 2/8/2021 */

/* 3/11/2021 */
/* USER BEGIN: callback for brake signal */
static void
cmnode_callback_brake(const std_msgs::Bool::ConstPtr &msg)
{
    if (msg->data)
    {
        DBW.brake = 1;
        LOG("brake");
    } else {
        DBW.brake = 0;
        LOG("No brake");
    }
}
/* USER END: callback for brake angle */
/* 3/11/2021 */

static double heading_cal(double angle_rad)
{
    //
    double hedg_arr = RAD2DEG(angle_rad);
    hedg_arr = 90 - hedg_arr;
	while (hedg_arr>360 || hedg_arr<0)
	{
		if (hedg_arr>360)
		{
			hedg_arr = hedg_arr - 360;
		}
		else if (hedg_arr<0)
		{
			hedg_arr = hedg_arr + 360;
		}		
	}

    // Truck heading
    return hedg_arr;
}
/*!
 * Description:
 * - Exemplary Service Callback for CarMaker Remote using ROS
 * - e.g. via rqt Service Caller or terminal "rosservice call ..."
 *
 *
 */
static bool
cmnode_HelloCM_CB_SrvCMRemote(cmrosutils::CMRemote::Request &req,
                              cmrosutils::CMRemote::Response &resp)
{

    int rv = -2;
    char sbuf[512];

    LOG("%s: Service '%s' was triggered with type='%s', msg='%s', data='%s'",
        ros::this_node::getName().c_str(),
        CMNode.Services.CMRemote.Srv.getService().c_str(),
        req.type.c_str(), req.msg.c_str(), req.data.c_str());

    /* Commands to CarMaker GUI
     * - Tcl commands!
     * - More information see "ProgrammersGuide"
     */
    if (strcasecmp("guicmd", req.type.c_str()) == 0)
    {
        /* Default: Evaluate command sent with message */
        if (strcasecmp("eval", req.msg.c_str()) == 0)
        {
            /* e.g. data = 'LoadTestRun CMRosIF/AdaptiveCruiseControl; StartSim' */
            rv = GuiCmd_Eval(req.data.c_str());
        }
        else
        {
            if (strcasecmp("start", req.msg.c_str()) == 0)
            {
                if (req.data.length() == 0)
                    rv = GuiCmd_Eval("LoadTestRun CMRosIF/AdaptiveCruiseControl; StartSim");
                else
                {
                    sprintf(sbuf, "%s; StartSim", req.data.c_str());
                    rv = GuiCmd_Eval(sbuf);
                }
            }
            if (strcasecmp("stop", req.msg.c_str()) == 0)
                rv = GuiCmd_Eval("StopSim");
        }

        /* Commands directly to CarMaker Executable
        * - Warning:
        *   - Information normally provided by CarMaker GUI might be missing
        */
    }
    else if (strcasecmp("cmd", req.type.c_str()) == 0)
    {
        if (strcasecmp("start", req.msg.c_str()) == 0)
        {
            if (req.data.length() == 0)
            {
                /* Most strings are normally provided by CarMaker GUI */
                SimStart(NULL, ros::this_node::getName().c_str(),
                         "CMRosIF/AdaptiveCruiseControl", NULL, NULL);
            }
            else
            {
                /* Most strings are normally provided by CarMaker GUI */
                SimStart(NULL, ros::this_node::getName().c_str(),
                         req.data.c_str(), NULL, NULL);
            }
        }
        if (strcasecmp("stop", req.msg.c_str()) == 0)
            SimStop2(0);
        rv = 0;
    }

    resp.res = rv;
    return true;
}

/*****************************************************************************/
/**********          C-Code for interfacing with CarMaker!          **********/
/*****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif
    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Get versions from shared library
    * - Set the returned Version to 0 if there is no dependency!
    * - Compatibility check should be done by calling procedure
    *   as early as possible(e.g. before CMRosIF_CMNode_Init()
    *
    * Arguments:
    * - CMRosIFVer = CMRosIF shared library version (User defined)
    *                - Initially filled with version of CMRosIF management library
    * - CMNumVer   = CarMaker version used for shared library at compile time (normally CM_NUMVER)
    *                - Initially filled with version of CMRosIF management library
    * - RosVersion = ROS version used for shared library at compile time (normally ROS_VERSION)
    *                - Initially filled with version requested by CMRosIF management library (0 if no request)
    *
    */
    int
    CMRosIF_CMNode_GetVersion(unsigned long *CMRosIFCMNodeNumVer,
                              unsigned long *CMNumVer,
                              unsigned long *RosNumVer)
    {

        *CMRosIFCMNodeNumVer = CMNODE_NUMVER;
        *CMNumVer = CM_NUMVER;
        *RosNumVer = ROS_VERSION;

        return 0;
    }

    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Basic Initialization
    * - e.g. create ROS Node, subscriptions, ...
    * - Return values
    *   - "rv <  0" = Error at initialization, CarMaker executable will stop
    *   - "rv >= 0" = Everything OK, CarMaker executable will continue
    *
    * Arguments:
    * - Argc/Argv  = Arguments normally provided to ROS executable are not be provided
    *                to CM executable directly, but can be set in Infofile for CMRosIF
    *                with key "Node.Args" in "Data/Config/CMRosIFParameters"
    *
    * - CMNodeName = Default CarMaker Node name
    *                - Can be parameterized in Infofile for CMRosIF
    *                - Final node name might be different (argument remapping, ...)
    *
    * - Inf        = Handle to CarMaker Infofile with parameters for this interface
    *                - Please note that pointer may change, e.g. before TestRun begins
    *
    * ToDo:
    * - Possible to create/initialize node/... before each TestRun start instead of CM startup?
    * - New Param_Get() function to read parameters from Infofile
    */
    int
    CMRosIF_CMNode_Init(int Argc, char **Argv, char *CMNodeName, struct tInfos *Inf)
    {

        int rv;
        bool rvb = false;
        char sbuf[512] = "";
        char keybuf[256] = "";
        ros::NodeHandlePtr node = NULL;
        ros::V_string names;

        LOG("Initialize CarMaker ROS Node");
        LOG("  -> Node Version = %05d", CMNODE_NUMVER);
        LOG("  -> ROS Version  = %05d", ROS_VERSION);
        LOG("  -> CM Version   = %05d", CM_NUMVER);

        /* ROS initialization. Name of Node might be different after remapping! */
        if (ros::isInitialized() == false)
        {
            /* "Remapping arguments" functionality (launchfiles, ...)? */
            ros::init(Argc, Argv, CMNodeName);
        }
        else
        {
            //node.reset(); ToDo!
        }

        if (ros::master::check() == false)
        {
            LogErrF(EC_Init, "Can't contact ROS Master!\n Start roscore or run launch file e.g. via Extras->CMRosIF\n");
            ros::shutdown();
            return -1;
        }

        /* Node specific */
        CMNode.Cfg.Ros.Node = ros::NodeHandlePtr(boost::make_shared<ros::NodeHandle>());
        node = CMNode.Cfg.Ros.Node;

        /* Publish specific */
        CMNode.Cfg.QueuePub = iGetIntOpt(Inf, "Node.QueuePub", 1000); /* ToDo: Influence of queue length relevant? */

        /* Prepare the node to provide simulation time. CarMaker will be /clock server */
        strcpy(sbuf, "/use_sim_time");

        if ((rv = node->hasParam(sbuf)) == true)
        {
            node->getParam(sbuf, rvb);
            LOG("  -> Has param '%s' with value '%d'", sbuf, rvb);
        }

        /* Additional switch to provide simulation Time */
        strcpy(keybuf, "Node.UseSimTime");

        if ((rv = iGetIntOpt(Inf, keybuf, 1)) > 0)
        {
            /* Parameter must be set before other nodes start
            * - set parameter outside to be independent from execution order?
            */
            LOG("  -> Provide simulation time!");
            //node->setParam("/use_sim_time", true); /* enable parameter if not already done */

            CMNode.Cfg.nCyclesClock = iGetIntOpt(Inf, "Node.nCyclesClock", 1000);

            strcpy(sbuf, "/clock");
            LOG("    -> Publish '%s' every %dms", sbuf, CMNode.Cfg.nCyclesClock);
            CMNode.Topics.Pub.Clock.Pub = node->advertise<rosgraph_msgs::Clock>(sbuf, CMNode.Cfg.QueuePub);

            /* ToDo: Necessary/Possible to ensure /clock is zeroed? */

            CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
            CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
        }
        else
        {
            LOG("  -> Don't provide simulation time!");
            CMNode.Cfg.nCyclesClock = 0;
        }

        strcpy(sbuf, hellocm::tpc_in_name.c_str() /*! Opposite in/out compared to external node */);
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.CM2Ext.Pub = node->advertise<hellocm_msgs::CM2Ext>(sbuf, CMNode.Cfg.QueuePub);
        CMNode.Topics.Pub.CM2Ext.Job = CMCRJob_Create("CM2Ext");
        CMNode.Topics.Pub.CM2Ext.CycleTime = 5000;
        CMNode.Topics.Pub.CM2Ext.CycleOffset = 0;

        /* USER BEGIN: Add LiDAR publisher */
        /*
        strcpy(sbuf, "/pandar");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.Lidar.Pub = node->advertise<sensor_msgs::PointCloud2>(sbuf, CMNode.Cfg.QueuePub);
        CMNode.Topics.Pub.Lidar.Job = CMCRJob_Create("pointcloud");
        CMNode.Topics.Pub.Lidar.CycleTime = 100;
        CMNode.Topics.Pub.Lidar.CycleOffset = 25;
        */
        /* USER END: Add LiDAR publisher */

        /* USER BEGIN: Add Image publisher*/
        strcpy(sbuf, "/camera/image_color/compressed");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.Image0.Pub = node->advertise<sensor_msgs::Image>(sbuf, CMNode.Cfg.QueuePub);
        CMNode.Topics.Pub.Image0.Job = CMCRJob_Create("camera");
        CMNode.Topics.Pub.Image0.CycleTime = 1000 / 25;
        CMNode.Topics.Pub.Image0.CycleOffset = 0;

        strcpy(sbuf, "/IPG_camera1");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.Image1.Pub = node->advertise<sensor_msgs::Image>(sbuf, CMNode.Cfg.QueuePub);
        /* USER END: Add Image publisher */

        /* USER BEGIN: Add Object publisher */
        strcpy(sbuf, "/ObjectList");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.Object.Pub = node->advertise<visualization_msgs::MarkerArray>(sbuf, CMNode.Cfg.QueuePub);
        CMNode.Topics.Pub.Object.Job = CMCRJob_Create("object");
        CMNode.Topics.Pub.Object.CycleTime = 100;
        CMNode.Topics.Pub.Object.CycleOffset = 125;
        /* USER END: Add Object publisher */

        /* USER BEGIN: Add IMU publisher */
        strcpy(sbuf, "/OdomIMU");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.OdomIMU.Pub = node->advertise<geometry_msgs::PoseStamped>(sbuf, CMNode.Cfg.QueuePub);
        CMNode.Topics.Pub.OdomIMU.Job = CMCRJob_Create("IMU");
        CMNode.Topics.Pub.OdomIMU.CycleTime = 10;
        CMNode.Topics.Pub.OdomIMU.CycleOffset = 0;
        /* USER END: Add IMU publisher */

        /* USER BEGIN: Add Path publisher */
        strcpy(sbuf, "/PathIMU");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.PathIMU.Pub = node->advertise<nav_msgs::Path>(sbuf, CMNode.Cfg.QueuePub);
        CMNode.Topics.Pub.PathIMU.Job = CMCRJob_Create("Path");
        CMNode.Topics.Pub.PathIMU.CycleTime = 10;
        CMNode.Topics.Pub.PathIMU.CycleOffset = 5;
        /* USER END: Add Path publisher */

        /* 2/2/2021 */
        /* USER BEGIN: Add GPS publisher */
        strcpy(sbuf, "/gpsfix_msg");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.GNSS_out.Pub = node->advertise<gps_common::GPSFix>(sbuf, CMNode.Cfg.QueuePub);
        CMNode.Topics.Pub.GNSS_out.Job = CMCRJob_Create("GNSS");
        CMNode.Topics.Pub.GNSS_out.CycleTime = 10;
        CMNode.Topics.Pub.GNSS_out.CycleOffset = 0;
        /* USER END: Add GPS publisher */
        /* 2/2/2021 */

        /* 2/8/2021 */
        /* USER BEGIN: Add vehicle status publisher */
        strcpy(sbuf, "/vhcl_out");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.vhcl_out.Pub = node->advertise<hellocm_msgs::CM2Ext_vhcl>(sbuf, CMNode.Cfg.QueuePub);
        CMNode.Topics.Pub.vhcl_out.Job = CMCRJob_Create("vhcl");
        CMNode.Topics.Pub.vhcl_out.CycleTime = 10;
        CMNode.Topics.Pub.vhcl_out.CycleOffset = 0;
        /* USER END: Add vehicle status publisher */
        /* 2/8/2021 */

        /* Subscribe specific */
        CMNode.Cfg.QueueSub = iGetIntOpt(Inf, "Node.QueueSub", 1); /* ToDo: Effect of queue length for subscriber? */

        strcpy(sbuf, hellocm::tpc_out_name.c_str() /*! Opposite in/out compared to external node */);
        LOG("  -> Subscribe '%s'", sbuf);
        CMNode.Topics.Sub.Ext2CM.Sub = node->subscribe(sbuf, CMNode.Cfg.QueueSub, cmnode_HelloCM_CB_TpcIn);
        CMNode.Topics.Sub.Ext2CM.Job = CMCRJob_Create("Ext2CM_for_Sync");

        /* In this example cycle time might be updated with value of external ROS Node
     * - See CMRosIF_CMNode_TestRun_Start_atBegin() */
        CMNode.Topics.Sub.Ext2CM.CycleTime = 15000;

        /* USER BEGIN: Add GPS subscriber */
        strcpy(sbuf, "/nav_msg"); // check topic name
        LOG("  -> Subscribe '%s'", sbuf);
        CMNode.Topics.Sub.GPSin.Sub = node->subscribe<sensor_msgs::NavSatFix>(sbuf, CMNode.Cfg.QueueSub, cmnode_callback_GPSin);
        CMNode.Topics.Sub.GPSin.Job = CMCRJob_Create("GPSin");
        CMNode.Topics.Sub.GPSin.CycleTime = 10;
        /* USER END: Add GPS subscriber */

        /* 2/8/2021 */
        /* USER BEGIN: Add steering_angle subscriber */
        strcpy(sbuf, "/steering_angle"); // check topic name
        LOG("  -> Subscribe '%s'", sbuf);
        CMNode.Topics.Sub.steering_angle.Sub = node->subscribe<std_msgs::Int16>(sbuf, CMNode.Cfg.QueueSub, cmnode_callback_steering);
        CMNode.Topics.Sub.steering_angle.Job = CMCRJob_Create("steering_angle");
        CMNode.Topics.Sub.steering_angle.CycleTime = 10;
        /* USER END: Add steering_angle subscriber */
        /* 2/8/2021 */

        /* 3/11/2021 */
        /* USER BEGIN: Add brake_signal subscriber */
        strcpy(sbuf, "/brake_signal"); // check topic name
        LOG("  -> Subscribe '%s'", sbuf);
        CMNode.Topics.Sub.brake_signal.Sub = node->subscribe<std_msgs::Bool>(sbuf, CMNode.Cfg.QueueSub, cmnode_callback_brake);
        CMNode.Topics.Sub.brake_signal.Job = CMCRJob_Create("brake_signal");
        CMNode.Topics.Sub.brake_signal.CycleTime = 10;
        /* USER END: Add brake_signal subscriber */
        /* 3/11/2021 */

        /* Services */
        strcpy(sbuf, hellocm::srv_init_name.c_str());
        LOG("  -> Service Client '%s'", sbuf);
        CMNode.Services.Init.Clnt = node->serviceClient<hellocm_msgs::Init>(sbuf);

        strcpy(sbuf, "CMRemote");
        LOG("  -> Create Service '%s'", sbuf);
        CMNode.Services.CMRemote.Srv = node->advertiseService(
            sbuf, cmnode_HelloCM_CB_SrvCMRemote);

        /* Print general information after everything is done */
        LOG("Initialization of ROS Node finished!");
        LOG("  -> Node Name = '%s'", ros::this_node::getName().c_str());
        LOG("  -> Namespace = '%s'", ros::this_node::getNamespace().c_str());

        /* Advertised Topics */
        ros::this_node::getAdvertisedTopics(names);
        LOG("  -> Advertised Topics (%lu)", names.size());

        auto it = names.begin();
        for (; it != names.end(); ++it)
            LOG("    -> %s", (*it).c_str());

        /* Subscribed Topics */
        names.clear();
        ros::this_node::getSubscribedTopics(names);
        LOG("  -> Subscribed Topics (%lu)", names.size());
        it = names.begin();
        for (; it != names.end(); ++it)
            LOG("    -> %s", (*it).c_str());

        return 1;
    }

    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Add user specific Quantities for data storage
    *   and visualization to DataDictionary
    * - Called once at program start
    * - no realtime conditions
    *
    */
    void
    CMRosIF_CMNode_DeclQuants(void)
    {

        tDDefault *df = DDefaultCreate("CMRosIF.");

        DDefULong(df, "CycleNoRel", "ms", &CMNode.CycleNoRel, DVA_None);
        DDefInt(df, "Sync.Cycles", "-", &CMNode.Sync.nCycles, DVA_None);
        DDefDouble(df, "Sync.Time", "s", &CMNode.Sync.Duration, DVA_None);
        DDefInt(df, "Sync.CyclePrepDone", "-", &CMNode.Sync.CyclePrepDone, DVA_None);
        DDefInt(df, "Sync.CycleJobDone", "-", &CMNode.Sync.CycleJobDone, DVA_None);
        DDefDouble4(df, "Sync.SynthDelay", "s", &CMNode.Sync.SynthDelay, DVA_IO_In);

        DDefUChar(df, "Cfg.Mode", "-", (unsigned char *)&CMNode.Cfg.Mode, DVA_None);
        DDefInt(df, "Cfg.nCyclesClock", "ms", &CMNode.Cfg.nCyclesClock, DVA_None);
        DDefChar(df, "Cfg.SyncMode", "-", (char *)&CMNode.Cfg.SyncMode, DVA_None);
        DDefDouble4(df, "Cfg.SyncTimeMax", "s", &CMNode.Cfg.SyncTimeMax, DVA_IO_In);

        DDefInt(df, "Mdl.CycleNo", "-", &CMNode.Model.CycleNo, DVA_None);
        DDefInt(df, "Mdl.CycleLastOut", "ms", &CMNode.Model.CycleLastOut, DVA_None);
        DDefInt(df, "Mdl.CycleLastIn", "ms", &CMNode.Model.CycleLastIn, DVA_None);
        DDefInt(df, "Mdl.CycleLastFlush", "ms", &CMNode.Model.CycleLastFlush, DVA_None);

        DDefaultDelete(df);
    }

    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Called when starting a new TestRun
    * - In separate Thread (no realtime conditions)
    * - After standard Infofiles are read in
    * - Return values
    *   - "rv <  0" = Error, TestRun start will be aborted
    *   - "rv >= 0" = Everything OK
    *
    * Arguments:
    * - Inf = CarMaker Infofile for CMRosIF with content after TestRun start
    *         - Please note that the Infofile provided at initialization might have been updated!
    *
    * ToDo:
    * - New Param_Get() function to read parameters from Infofile
    *
    */
    int
    CMRosIF_CMNode_TestRun_Start_atBegin(struct tInfos *Inf)
    {

        /* Node can be disabled via Infofile */
        tCMNode_Mode *pmode = &CMNode.Cfg.Mode;
        tCMNode_SyncMode *psyncmode = &CMNode.Cfg.SyncMode;

        if (Inf != NULL)
        {
            *pmode = (tCMNode_Mode)iGetIntOpt(Inf, "Node.Mode", CMNode_Mode_Disabled);
            *psyncmode = (tCMNode_SyncMode)iGetIntOpt(Inf, "Node.Sync.Mode", CMNode_SyncMode_Disabled);
        }

        if (/*SimCore.CycleNo == 0 ||*/ Inf == NULL || *pmode == CMNode_Mode_Disabled)
        {
            *pmode = CMNode_Mode_Disabled;
            LOG("CarMaker ROS Node is disabled!");
            return 0;
        }

        char sbuf[512];
        char key[256];
        char *str = NULL;
        int rv = 0;
        bool rvb = false;

        int cycletime = 0;
        int *pcycletime = NULL;
        int cycleoff = 0;
        tCMCRJob *job = NULL;
        auto srv = &CMNode.Services.Init;

        LOG("CarMaker ROS Node is enabled! Mode=%d, SyncMode=%d", *pmode, *psyncmode);
        LOG("  -> Node Name = %s", ros::this_node::getName().c_str());

        /* Update synchronization */
        if (*psyncmode != CMNode_SyncMode_Disabled && *psyncmode != CMNode_SyncMode_Tpc)
        {
            LogErrF(EC_Sim, "CMNode: Invalid synchronization mode '%d'!", *psyncmode);
            *pmode = CMNode_Mode_Disabled;
            return -1;
        }

        CMNode.Cfg.SyncTimeMax = iGetDblOpt(Inf, "Node.Sync.TimeMax", 1.0);

        /* Reset for next cycle */
        CMNode.CycleNoRel = 0;
        CMNode.Sync.Duration = 0.0;
        CMNode.Sync.nCycles = -1;
        CMNode.Sync.CycleJobDone = -1;
        CMNode.Sync.CyclePrepDone = -1;
        CMNode.Model.CycleNo = -1;
        CMNode.Model.CycleLastIn = -1;
        CMNode.Model.CycleLastOut = -1;
        CMNode.Model.CycleLastFlush = -1;

        /* Allow an update of the clock only if it was enabled before! */
        if (CMNode.Cfg.nCyclesClock > 0)
        {
            if ((rv = iGetIntOpt(Inf, "Node.nCyclesClock", 1000)) > 0)
                CMNode.Cfg.nCyclesClock = rv;
        }

        /* Necessary to ensure /clock is zeroed here?
        * ToDo: Create function? */
        if (CMNode.Cfg.nCyclesClock > 0)
        {
            LOG("  -> Publish /clock every %dms", CMNode.Cfg.nCyclesClock);
            CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
            CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
        }

        /* Prepare external node for next simulation */
        if (!srv->Clnt.exists())
        {
            // ToDo: possible to get update if external ROS Node name changes?
            LogErrF(EC_Sim, "ROS Service is not ready! Please start external ROS Node providing Service '%s'!",
                    srv->Clnt.getService().c_str());
            *pmode = CMNode_Mode_Disabled;
            return -1;
        }

        LOG("  -> Send Service Request");

        /* ToDo: Async?*/
        if (!srv->Clnt.call(srv->Msg))
        {
            LogErrF(EC_Sim, "ROS Service error!");
            *pmode = CMNode_Mode_Disabled;
            return -1;
        }

        /* Update cycle time with information of external node */
#if 1
        /* Variant 1:
        * - Receiving parameters via ROS Parameter Server
        * - Parameter may be set externally e.g. by other node or arguments to command
        * - ROS parameters are more flexible than ROS services!
        */
        strcpy(sbuf, hellocm::prm_cycletime_name.c_str());
        if ((rv = CMNode.Cfg.Ros.Node->hasParam(sbuf)) == true)
            CMNode.Cfg.Ros.Node->getParam(sbuf, rv);
#else
        /* Variant 2:
        * - Receiving parameters from external Node via Service
        * - Services might be too "static"
        * - Not recommended!
        */
        rv = srv->Msg.response.cycletime;
#endif

        pcycletime = &CMNode.Topics.Sub.Ext2CM.CycleTime;

        if (*pcycletime != rv)
        {
            LOG("  -> Cycle time of external node changed from %dms to %dms", *pcycletime, rv);
            *pcycletime = rv;
        }

        /* Plausibility check for Cycle Time */
        if (CMNode.Cfg.nCyclesClock > 0 && (*pcycletime < CMNode.Cfg.nCyclesClock || *pcycletime % CMNode.Cfg.nCyclesClock != 0))
        {

            LogErrF(EC_Sim, "Ext. ROS Node has invalid cycle time! Expected multiple of %dms but got %dms",
                    CMNode.Cfg.nCyclesClock, *pcycletime);

            *pmode = CMNode_Mode_Disabled;
            return -1;
        }

        /* USER BEGIN: Get LiDAR parameters */
        tInfos *Inf_Sensor = nullptr;
        tErrorMsg *err = nullptr;
        Inf_Sensor = InfoNew();

        /****** Velodyne 32 channel LiDAR ******/
        // iRead2(&err, Inf_Sensor, "Data/Sensor/LidarRSI_VLP32C", "LidarCode");

        // CMNode.LidarRSI.ret = iGetTableOpt(Inf_Sensor, "Beams", CMNode.LidarRSI.BeamTable, tablesize, 6, &CMNode.LidarRSI.rows);
        // CMNode.LidarRSI.FOV_h = iGetDblOpt(Inf_Sensor, "Beams.FOVH", 3) * (-2);
        // CMNode.LidarRSI.FOV_v = iGetDblOpt(Inf_Sensor, "Beams.FOVV", 4) * (-2);
        // CMNode.LidarRSI.nBeams = iGetFixedTable2(Inf_Sensor, "Beams.N", 2, 1);
        // CMNode.LidarRSI.nBeams_h = CMNode.LidarRSI.nBeams[0];
        // CMNode.LidarRSI.nBeams_v = CMNode.LidarRSI.nBeams[1];        

        tInfos *Inf_Vehicle = nullptr;
        Inf_Vehicle = InfoNew();
        const char *FName;
        FName = InfoGetFilename(SimCore.Vhcl.Inf);
        Log("FName = %s\n", FName);
        iRead2(&err, Inf_Vehicle, FName, "SensorReadCode");
        
        /*
        CMNode.LidarRSI.position = iGetFixedTable2(Inf_Vehicle, "Sensor.LidarRSI.0.pos", 3, 1);
        CMNode.LidarRSI.pos.x = CMNode.LidarRSI.position[0];
        CMNode.LidarRSI.pos.y = CMNode.LidarRSI.position[1];
        CMNode.LidarRSI.pos.z = CMNode.LidarRSI.position[2];

        CMNode.LidarRSI.rotation = iGetFixedTable2(Inf_Vehicle, "Sensor.LidarRSI.0.rot", 3, 1);
        CMNode.LidarRSI.rot.x = CMNode.LidarRSI.rotation[0];
        CMNode.LidarRSI.rot.y = CMNode.LidarRSI.rotation[1];
        CMNode.LidarRSI.rot.z = CMNode.LidarRSI.rotation[2];

        CMNode.LidarRSI.mountFrame = iGetStrOpt(Inf_Vehicle, "Sensor.LidarRSI.0.Mounting", "/Fr1A");
        CMNode.LidarRSI.cycleTime = iGetDblOpt(Inf_Vehicle, "Sensor.LidarRSI.0.CycleTime", 100);
        CMNode.LidarRSI.cycleOffset = iGetDblOpt(Inf_Vehicle, "Sensor.LidarRSI.0.nCycleOffset", 0);
        
        // ****** Hesai Pandar 64 channel LiDAR ******
        char *sensor_dir = "Data/Sensor/";
        char *LiDAR_name = iGetStr(Inf_Vehicle, "Sensor.LidarRSI.0.BeamsFName");
        char *LiDAR_dir = new char[strlen(sensor_dir) + strlen(LiDAR_name) + 1];
        strcpy(LiDAR_dir, sensor_dir);
        strcat(LiDAR_dir, LiDAR_name);
        Log("LiDAR_dir = %s\n", LiDAR_dir);

        iRead2(&err, Inf_Sensor, LiDAR_dir, "LidarCode");

        CMNode.LidarRSI.ret = iGetTableOpt(Inf_Sensor, "Beams", CMNode.LidarRSI.BeamTable, tablesize, 6, &CMNode.LidarRSI.rows);
        CMNode.LidarRSI.FOV_h = iGetDblOpt(Inf_Sensor, "Beams.FoVH", 3) * (-2);
        CMNode.LidarRSI.FOV_v = iGetDblOpt(Inf_Sensor, "Beams.FoVV", 4) * (-2);
        CMNode.LidarRSI.nBeams = iGetFixedTable2(Inf_Sensor, "Beams.N", 2, 1);
        CMNode.LidarRSI.nBeams_h = CMNode.LidarRSI.nBeams[0];
        CMNode.LidarRSI.nBeams_v = CMNode.LidarRSI.nBeams[1];
        // Log("FOV_h = %f\n", CMNode.LidarRSI.FOV_h);
        // Log("FOV_v = %f\n", CMNode.LidarRSI.FOV_v);
        Log("nBeams_h = %f\n", CMNode.LidarRSI.nBeams[0]);
        Log("nBeams_v = %f\n", CMNode.LidarRSI.nBeams[1]);
        */
        /* USER END: Get LiDAR parameters */

        /* USER BEGIN: Get object parameters */
        int VehicleInfo_Err = iRead2(&err, Inf_Vehicle, FName, "SenserReadCode");

        if (VehicleInfo_Err == 0)
        {
            CMNode.ObjectSensor.position = iGetFixedTable2(Inf_Vehicle, "Sensor.2.pos", 3, 1);
            CMNode.ObjectSensor.pos.x = CMNode.ObjectSensor.position[0];
            CMNode.ObjectSensor.pos.y = CMNode.ObjectSensor.position[1];
            CMNode.ObjectSensor.pos.z = CMNode.ObjectSensor.position[2];

            CMNode.ObjectSensor.rotation = iGetFixedTable2(Inf_Vehicle, "Sensor.2.rot", 3, 1);
            CMNode.ObjectSensor.rot.x = CMNode.ObjectSensor.rotation[0];
            CMNode.ObjectSensor.rot.y = CMNode.ObjectSensor.rotation[1];
            CMNode.ObjectSensor.rot.z = CMNode.ObjectSensor.rotation[2];

            CMNode.ObjectSensor.mountFrame = iGetStrOpt(Inf_Vehicle, "Sensor.2.Mounting", "/Fr1A");
            CMNode.ObjectSensor.cycleTime = 1 / iGetDblOpt(Inf_Vehicle, "Sensor.Param.2.UpdRate", 100);
            CMNode.ObjectSensor.cycleOffset = iGetDblOpt(Inf_Vehicle, "Sensor.Param.2.CycleOffset = 0", 0);
        }
        /* USER END: Get object parameters */

        /* 2/3/2021 */
        /* USER BEGIN: Get GNSS parameters */
        int GNSS_Err = iRead2(&err, Inf_Vehicle, FName, "SenserReadCode");

        if (GNSS_Err == 0)
        {
            CMNode.GNSS.position = iGetFixedTable2(Inf_Vehicle, "Sensor.1.pos", 3, 1);
            CMNode.GNSS.pos.x = CMNode.GNSS.position[0];
            CMNode.GNSS.pos.y = CMNode.GNSS.position[1];
            CMNode.GNSS.pos.z = CMNode.GNSS.position[2];

            CMNode.GNSS.mountFrame = iGetStrOpt(Inf_Vehicle, "Sensor.1.Mounting", "/Fr1A");
            CMNode.GNSS.cycleTime = 1 / iGetDblOpt(Inf_Vehicle, "Sensor.Param.1.UpdRate", 100);
            CMNode.GNSS.cycleOffset = iGetDblOpt(Inf_Vehicle, "Sensor.Param.2.CycleOffset", 0);
        }
        /* USER END: Get GNSS parameters */
        /* 2/3/2021 */

        /* Prepare Jobs for publish and subscribe
        * - Special use case:
        *   - Topic in and Topic out use same cycle time with relative shift!
        */

        /* Start to publish when simulation starts */
        job = CMNode.Topics.Pub.CM2Ext.Job;
        cycletime = CMNode.Topics.Pub.CM2Ext.CycleTime;
        cycleoff = CMNode.Topics.Pub.CM2Ext.CycleOffset;

        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);

        /* USER BEGIN: camera */
        job = CMNode.Topics.Pub.Image0.Job;
        cycletime = CMNode.Topics.Pub.Image0.CycleTime;
        cycleoff = CMNode.Topics.Pub.Image0.CycleOffset;

        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
        /* USER END: camera */

        /* USER BEGIN: LiDAR sensor */
        /*
        job = CMNode.Topics.Pub.Lidar.Job;
        cycletime = CMNode.Topics.Pub.Lidar.CycleTime;
        cycleoff = CMNode.Topics.Pub.Lidar.CycleOffset;

        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
        */
        /* USER END: LiDAR sensor */

        /* USER BEGIN: Object sensor */
        job = CMNode.Topics.Pub.Object.Job;
        cycletime = CMNode.Topics.Pub.Object.CycleTime;
        cycleoff = CMNode.Topics.Pub.Object.CycleOffset;

        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
        /* USER END: Object sensor */

        /* USER BEGIN: IMU sensor */
        job = CMNode.Topics.Pub.OdomIMU.Job;
        cycletime = CMNode.Topics.Pub.OdomIMU.CycleTime;
        cycleoff = CMNode.Topics.Pub.OdomIMU.CycleOffset;

        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
        /* USER END: IMU sensor */

        /* USER BEGIN: IMU sensor */
        job = CMNode.Topics.Pub.PathIMU.Job;
        cycletime = CMNode.Topics.Pub.PathIMU.CycleTime;
        cycleoff = CMNode.Topics.Pub.PathIMU.CycleOffset;

        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
        /* USER END: IMU sensor */

        /* 2/2/2021 */
        /* USER BEGIN: GPS sensor */
        job = CMNode.Topics.Pub.GNSS_out.Job;
        cycletime = CMNode.Topics.Pub.GNSS_out.CycleTime;
        cycleoff = CMNode.Topics.Pub.GNSS_out.CycleOffset;

        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
        /* USER END: GPS sensor */
        /* 2/2/2021 */

        /* 2/8/2021 */
        /* USER BEGIN: Vehicle status */
        job = CMNode.Topics.Pub.vhcl_out.Job;
        cycletime = CMNode.Topics.Pub.vhcl_out.CycleTime;
        cycleoff = CMNode.Topics.Pub.vhcl_out.CycleOffset;

        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
        /* USER END: Vehicle status */
        /* 2/8/2021 */

/* VDS Init*/
/*
#ifdef VDS
        std::vector<ros::Publisher> Pub_vec = {CMNode.Topics.Pub.Image0.Pub,
                                               CMNode.Topics.Pub.Image1.Pub};
        VDS_Init(Pub_vec);
        LOG("Initialization of VDS finished!");
#endif
*/
/* USER END: VDS Init*/
        /* Synchronization with external node
        * - external node provides cycle time (see service above)
        * - other parameterization methods (e.g. ROS parameter, ...) are possible!
        * - Expect sync Topic are delayed (communication time, ...)
        * - This example shows sync via ROS Timer in external node
        *   - Therefore "/clock" topic needs to be published by CarMaker!
        *   - Other mechanism, e.g. data triggered on external node side
        *     via publishing Topic directly inside subscription callback is also possible!
        * - time=0.0 can't be detected by external node, therefore
        *   first receive needs to start after expected cycle time
        *   of external ROS node
        */

        job = CMNode.Topics.Sub.Ext2CM.Job;
        cycletime = CMNode.Topics.Sub.Ext2CM.CycleTime;
        cycleoff = CMNode.Topics.Sub.Ext2CM.CycleOffset = 0; /* No offset allowed if ROS Timer is used for sync!*/

        /* Create the synchronization jobs */
        if (*psyncmode == CMNode_SyncMode_Tpc)
        {
            CMCRJob_Init(job, cycletime + 1, cycletime, CMCRJob_Mode_Ext);

            LOG("  -> Synchronize on Topic '%s' (cycletime=%d, cycleoff=%d)",
                CMNode.Topics.Sub.Ext2CM.Sub.getTopic().c_str(), cycletime, cycleoff);
        }
        else
            CMCRJob_Init(job, cycletime + 1, cycletime, CMCRJob_Mode_Default);

        /* USER BEGIN: GPSin sensor */
        job = CMNode.Topics.Sub.GPSin.Job;
        cycletime = CMNode.Topics.Sub.GPSin.CycleTime;
        cycleoff = CMNode.Topics.Sub.GPSin.CycleOffset;

        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
        /* USER END: GPSin sensor */

        /* 2/8/2021 */
        /* USER BEGIN: steering_angle */
        job = CMNode.Topics.Sub.steering_angle.Job;
        cycletime = CMNode.Topics.Sub.steering_angle.CycleTime;
        cycleoff = CMNode.Topics.Sub.steering_angle.CycleOffset;

        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
        /* USER END: steering_angle */
        /* 2/8/2021 */

        /* 3/11/2021 */
        /* USER BEGIN: brake_signal */
        job = CMNode.Topics.Sub.brake_signal.Job;
        cycletime = CMNode.Topics.Sub.brake_signal.CycleTime;
        cycleoff = CMNode.Topics.Sub.brake_signal.CycleOffset;

        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
        /* USER END: brake_signal */
        /* 3/11/2021 */

        LOG("External ROS Node is ready to simulate");

        return 1;
    }

    /*!
    * ToDo:
    * - Put everything to TestRun_Start_atBegin?
    *
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Repeating call for several CarMaker cycles until return value is 1
    * - May be called even previous return value was 1
    * - See "User.c:User_TestRun_RampUp()"
    *
    */
    int
    CMRosIF_CMNode_TestRun_RampUp(void)
    {
        LOG("CMRosIF_CMNode_TestRun_RampUp");
        /* Return immediately if node is disabled */
        if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
            return 1;

        /* Put your code here */
        
        return 1;
    }

    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Called when TestRun ends (no realtime conditions)
    * - See "User.c:User_TestRun_End()"
    *
    */
    int
    CMRosIF_CMNode_TestRun_End(void)
    {

        LOG("CMRosIF_CMNode_TestRun_End");
        /* Put your code here */
        /* USER BEGIN: VDS Exit */
/*
#ifdef VDS
        VDS_Exit();
#endif
*/
        /* USER END: VDS Exit */
        /* Disable after simulation has finished */
        CMNode.Cfg.Mode = CMNode_Mode_Disabled;

        return 1;
    }

    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Called at very beginning of CarMaker cycle
    * - Process all topics/services using different modes or synchronization modes
    * - See "User.c:User_In()"
    *
    * ToDo:
    * - Additional spin mechanism
    *   - e.g. for HIL
    *   - e.g. spinning in new thread, copying incoming data here, ...
    *
    */
    int
    CMRosIF_CMNode_In(void)
    {
        int rv = 0;
        int rx_done = 0;
        const char *job_name = NULL;
        tCMCRJob *job = NULL;
        ros::WallTime tStart = ros::WallTime::now();
        ros::WallDuration tDelta = ros::WallDuration(0.0);
        CMNode.Sync.nCycles = 0;
        CMNode.Sync.Duration = 0.0;

        switch (CMNode.Cfg.Mode)
        {
        case CMNode_Mode_Disabled:
            /* Comment next line if no messages/services
	     * shall be processed in disabled Node state
	     */
            ros::spinOnce();
            break;

        case CMNode_Mode_Default:

            if (CMNode.Cfg.SyncMode != CMNode_SyncMode_Tpc)
            {
                /* Process messages in queue, but do not block */
                ros::spinOnce();
            }
            else
            {
                /* Synchronization based on expected Topics
                * - Blocking call (process publish and wait for answer)
                * - Stop simulation if maximum time is exceeded
                */
                do
                {
                    ros::spinOnce();

                    /* Only do anything if simulation is running */
                    if (SimCore.State != SCState_Simulate)
                    {
                        rx_done = 1;
                        break;
                    }

                    rx_done = 0;

                    /* Check all jobs if preparation is done */
                    job = CMNode.Topics.Sub.Ext2CM.Job;

                    if ((rv = CMCRJob_DoPrep(job, CMNode.CycleNoRel, 0, NULL, NULL)) < CMCRJob_RV_OK)
                    {
                        LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(job), CMCRJob_RVStr(rv));
                        rx_done = 0;
                        break;
                    }

                    /* If job is not done, remember name and prevent loop to finish */
                    job_name = (rv != CMCRJob_RV_DoSomething ? NULL : CMCRJob_GetName(job));
                    rx_done = rv == CMCRJob_RV_DoNothing ? 1 : 0;

                    if (rx_done == 1)
                        break;

                    /* Wait a little that data can arrive. WallTime, NOT ROS time!!!*/
                    ros::WallDuration(0.0).sleep();
                    tDelta = ros::WallTime::now() - tStart;
                    CMNode.Sync.nCycles++;

                } while (ros::ok() && rx_done == 0 && tDelta.toSec() < CMNode.Cfg.SyncTimeMax);

                /* Final calculation to get duration including last cycle before receive */
                tDelta = ros::WallTime::now() - tStart;

                CMNode.Sync.Duration = tDelta.toSec();

                if (rx_done != 1 && CMNode.Cfg.SyncTimeMax > 0.0 && tDelta.toSec() >= CMNode.Cfg.SyncTimeMax)
                    LogErrF(EC_Sim, "CMNode: Synchronization error! tDelta=%.3f, Last invalid Job='%s'\n", tDelta.toSec(), job_name);
            }

            break;

        case CMNode_Mode_Threaded:
            /* ToDo
	     * - Spinning in parallel thread started before
	     * - Lock variables!
	     * - e.g. for HIL
	     */
            break;

        default:
            /* Invalid!!! */;
        }

        return 1;
    }

    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Called after driving maneuver calculation
    * - before CMRosIF_CMNode_VehicleControl_Calc()
    * - See "User.c:User_DrivManCalc()"
    */
    int
    CMRosIF_CMNode_DrivMan_Calc(double dt)
    {
        /* Only do anything if simulation is running */
        if (CMNode.Cfg.Mode == CMNode_Mode_Disabled || SimCore.State != SCState_Simulate)
            return 0;

        /* Put your code here */
        // if (DBW.GolableEnable)
        // {
        if (DBW.brake)
        {
            LOG("DVA Brake Active");
            DVA_WriteRequest("DM.Gas", OWMode_Abs, 1000,0,0,0,NULL);
            DVA_WriteRequest("DM.Brake", OWMode_Abs, 1000,0,0,DBW.brake,NULL);
        }
        
        //    DVA_WriteRequest("DM.Gas", OWMode_Abs, 1000,0,0,DBW.throttle,NULL);
        //    DVA_WriteRequest("DM.Brake", OWMode_Abs, 1000,0,0,DBW.brake,NULL);
            DVA_WriteRequest("DM.Steer.Ang", OWMode_Abs, 1000,0,0,DBW.steering,NULL);
        //     // DVA_WriteRequest("DM.GearNo", OWMode_Abs, 1000,0,0,DBW.gear,NULL);
        // }     

        return 1;
    }

    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Called after CMRosIF_CMNode_DrivManCalc
    * - before CMRosIF_CMNode_VehicleControl_Calc()
    * - See "User.c:User_VehicleControl_Calc()"
    */
    int
    CMRosIF_CMNode_VehicleControl_Calc(double dt)
    {
        /* Only do anything if simulation is running */
        if (CMNode.Cfg.Mode == CMNode_Mode_Disabled || SimCore.State != SCState_Simulate)
            return 0;

        /* Put your code here */
        return 1;
    }

    bool odom_flag = false;
    int x = 0;
    int y = 0;
    int z = 0;
    double last_x = 0;
    double last_y = 0;
    double last_z = 0;
    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Called after vehicle model has been calculated
    * - See "User.c:User_Calc()"
    */
    int CMRosIF_CMNode_Calc(double dt)
    {
        /* Only do anything if simulation is running */
        if (CMNode.Cfg.Mode == CMNode_Mode_Disabled || SimCore.State != SCState_Simulate)
            return 0;

        /* Update model with values from external node only in specific cycle?
        * - This data handling is optionl, but necessary for deterministic behaviour
        * - if synchronization is active, incoming data remains in msg buffer until correct cycle
        */
        int rv;
        auto sync = &CMNode.Topics.Sub.Ext2CM;

        if ((rv = CMCRJob_DoJob(sync->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething)
        {
            LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sync->Job), CMCRJob_RVStr(rv));
        }
        else if (rv == CMCRJob_RV_DoSomething)
        {
            /* Something to do in sync cycle? */
            //CMCRJob_Info(in->Job, CMNode.CycleNoRel, "CMNode: Do Something for Sync: ");

            /* Update model parameters here? */
            CMNode.Model.CycleNo = CMNode.Topics.Sub.Ext2CM.Msg.cycleno;

            /* Remember cycle for debugging */
            CMNode.Sync.CycleJobDone = CMNode.CycleNoRel;
            CMNode.Model.CycleLastFlush = CMNode.CycleNoRel;
        }

        /* Do some calculation... */
        /* USER BEGIN: add Lidar Calculations using PCL to deal with pointcloud*/
/*         auto sync_Lidar = &CMNode.Topics.Pub.Lidar;

        if ((rv = CMCRJob_DoJob(sync_Lidar->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething)
        {
            LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sync_Lidar->Job), CMCRJob_RVStr(rv));
        }
        else if (rv == CMCRJob_RV_DoSomething)
        {
            pcl::PointCloud<pcl::PointXYZI> PointCloudFrame;
            pcl::PointXYZI point;
            int index = 0;

            index = LidarRSI_FindIndexForName("LIRSfront");
            if (index != -1)
            {
                for (int i = 0; i < LidarRSI[index].nScanPoints; i++)
                {
                    const int beam_id = LidarRSI[index].ScanPoint[i].BeamID;
                    const double azimuth = angles::from_degrees(CMNode.LidarRSI.BeamTable[4 * CMNode.LidarRSI.rows + beam_id]);
                    const double elevation = angles::from_degrees(CMNode.LidarRSI.BeamTable[5 * CMNode.LidarRSI.rows + beam_id]);
                    const double ray_length = 0.5 * LidarRSI[index].ScanPoint[i].LengthOF;
                    // LOG("LidarRSI[%i] = %i; i = %i;",index,LidarRSI[index].nScanPoints,i);

                    // point.x = ray_length * cos(elevation) * cos(azimuth);
                    // point.y = ray_length * cos(elevation) * sin(azimuth);
                    // point.z = ray_length * sin(elevation);
                    point.y = -ray_length * cos(elevation) * cos(azimuth);
                    point.x = ray_length * cos(elevation) * sin(azimuth);
                    point.z = ray_length * sin(elevation);

                    point.intensity = LidarRSI[index].ScanPoint[i].Intensity;
                    PointCloudFrame.push_back(point);
                }
            }
#ifdef perception
            // For Perception 
            index = LidarRSI_FindIndexForName("LIRSfront2");
            if (index != -1)
            {
                for (int i = 0; i < LidarRSI[index].nScanPoints; i++)
                {
                    const int beam_id = LidarRSI[index].ScanPoint[i].BeamID;
                    const double azimuth = angles::from_degrees(CMNode.LidarRSI.BeamTable[4 * CMNode.LidarRSI.rows + beam_id]);
                    const double elevation = angles::from_degrees(CMNode.LidarRSI.BeamTable[5 * CMNode.LidarRSI.rows + beam_id]);
                    const double ray_length = 0.5 * LidarRSI[index].ScanPoint[i].LengthOF;

                    point.x = ray_length * cos(elevation) * cos(azimuth) + 5.6;
                    point.y = ray_length * cos(elevation) * sin(azimuth);
                    point.z = ray_length * sin(elevation);
                    point.intensity = LidarRSI[index].ScanPoint[i].Intensity;
                    PointCloudFrame.push_back(point);
                }
            }

            index = LidarRSI_FindIndexForName("LIRSfront");
            if (index != -1)
            {
                for (int i = 0; i < LidarRSI[index].nScanPoints; i++)
                {
                    const int beam_id = LidarRSI[index].ScanPoint[i].BeamID;
                    const double azimuth = angles::from_degrees(CMNode.LidarRSI.BeamTable[4 * CMNode.LidarRSI.rows + beam_id]);
                    const double elevation = angles::from_degrees(CMNode.LidarRSI.BeamTable[5 * CMNode.LidarRSI.rows + beam_id]);
                    const double ray_length = 0.5 * LidarRSI[index].ScanPoint[i].LengthOF;

                    point.x = ray_length * cos(elevation) * cos(azimuth) + 5.6;
                    point.y = ray_length * cos(elevation) * sin(azimuth);
                    point.z = ray_length * sin(elevation) - 2.0;
                    point.intensity = LidarRSI[index].ScanPoint[i].Intensity;
                    PointCloudFrame.push_back(point);
                }
            }

            index = LidarRSI_FindIndexForName("LIRSleft");
            if (index != -1)
            {
                for (int i = 0; i < LidarRSI[index].nScanPoints; i++)
                {
                    const int beam_id = LidarRSI[index].ScanPoint[i].BeamID;
                    const double azimuth = angles::from_degrees(CMNode.LidarRSI.BeamTable[4 * CMNode.LidarRSI.rows + beam_id]);
                    const double elevation = angles::from_degrees(CMNode.LidarRSI.BeamTable[5 * CMNode.LidarRSI.rows + beam_id]);
                    const double ray_length = 0.5 * LidarRSI[index].ScanPoint[i].LengthOF;

                    point.y = ray_length * cos(elevation) * cos(azimuth) + 1.3;
                    point.x = -(ray_length * cos(elevation) * sin(azimuth)) + 3;
                    point.z = ray_length * sin(elevation) + 0.7;
                    point.intensity = LidarRSI[index].ScanPoint[i].Intensity;
                    PointCloudFrame.push_back(point);
                }
            }

            index = LidarRSI_FindIndexForName("LIRSright");
            if (index != -1)
            {
                for (int i = 0; i < LidarRSI[index].nScanPoints; i++)
                {
                    const int beam_id = LidarRSI[index].ScanPoint[i].BeamID;
                    const double azimuth = angles::from_degrees(CMNode.LidarRSI.BeamTable[4 * CMNode.LidarRSI.rows + beam_id]);
                    const double elevation = angles::from_degrees(CMNode.LidarRSI.BeamTable[5 * CMNode.LidarRSI.rows + beam_id]);
                    const double ray_length = 0.5 * LidarRSI[index].ScanPoint[i].LengthOF;

                    point.y = -(ray_length * cos(elevation) * cos(azimuth)) - 1.3;
                    point.x = (ray_length * cos(elevation) * sin(azimuth)) + 3;
                    point.z = ray_length * sin(elevation) + 0.7;
                    point.intensity = LidarRSI[index].ScanPoint[i].Intensity;
                    PointCloudFrame.push_back(point);
                }
            }
#endif
            pcl::toROSMsg(PointCloudFrame, CMNode.Topics.Pub.Lidar.Msg);
            CMNode.Topics.Pub.Lidar.Msg.header.frame_id = "Fr1A";
            CMNode.Topics.Pub.Lidar.Msg.header.stamp = ros::Time(LidarRSI[index].ScanTime);
            CMNode.Topics.Pub.Lidar.Pub.publish(CMNode.Topics.Pub.Lidar.Msg);
        }
        */        
        
        // LOG("LiDAR: %d, time: %ld", rv, CMNode.CycleNoRel);
        /* USER END: add Lidar Calculations using PCL to deal with pointcloud*/

        /* USER BEGIN: add Objects into MarkerArray*/
        auto sync_Object = &CMNode.Topics.Pub.Object;

        if ((rv = CMCRJob_DoJob(sync_Object->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething)
        {
            LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sync_Object->Job), CMCRJob_RVStr(rv));
        }
        else if (rv == CMCRJob_RV_DoSomething)
        {
            visualization_msgs::Marker markers;
            CMNode.Topics.Pub.Object.Msg.markers.clear();
            ros::Time object_sensor_stamp = ros::Time(ObjectSensor->TimeStamp);

            ros::Duration object_viz_time = ros::Duration(0.11);
            for (int j = 0; j < ObjectSensor->nObsvObjects; ++j)
            {
                int Global_ID = ObjectSensor->ObsvObjects[j];
                tObjectSensorObj *pOSO = ObjectSensor_GetObjectByObjId(0, Global_ID);
                if (!pOSO)
                    continue;
                markers.header.frame_id = "Fr1A";
                markers.id = pOSO->ObjId;
                markers.type = visualization_msgs::Marker::CUBE;
                markers.lifetime = object_viz_time;
                markers.header.stamp = object_sensor_stamp;

                //object scale
                markers.scale.x = pOSO->l;
                markers.scale.y = pOSO->w;
                markers.scale.z = pOSO->h;

                //object color
                if (markers.id >= 16000022)
                {
                    markers.color.r = 1.0;
                    markers.color.g = 0.0;
                    markers.color.b = 0.0;
                }
                else
                {
                    markers.color.r = 0.0;
                    markers.color.g = 1.0;
                    markers.color.b = 1.0;
                }

                if (pOSO->dtct)
                {
                    markers.color.a = 0.7;
                }
                else
                {
                    markers.color.a = 0.2;
                }

                //object location
                tf2::Quaternion rotation;
                rotation.setRPY(pOSO->RefPnt.r_zyx[0], pOSO->RefPnt.r_zyx[1], pOSO->RefPnt.r_zyx[2]);

                tf2::Vector3 obj_center = tf2::quatRotate(rotation, tf2::Vector3(0.5 * pOSO->l, 0, 0));

                // markers.pose.position.x = obj_center.getX() + pOSO->RefPnt.ds[0];
                // markers.pose.position.y = obj_center.getY() + pOSO->RefPnt.ds[1];
                markers.pose.position.y = -(obj_center.getX() + pOSO->RefPnt.ds[0]);
                markers.pose.position.x = obj_center.getY() + pOSO->RefPnt.ds[1];
                markers.pose.position.z = obj_center.getZ() + pOSO->RefPnt.ds[2];

                tf2::Quaternion q_rot;

                double r=0, p=0, y=M_PI/2;  // Rotate the previous pose by 90* about z
                q_rot.setRPY(r, p, y);

                rotation = q_rot*rotation;  // Calculate the new orientation
                rotation.normalize();
                markers.pose.orientation = tf2::toMsg(rotation);

                CMNode.Topics.Pub.Object.Msg.markers.push_back(markers);
            }
            CMNode.Topics.Pub.Object.Pub.publish(CMNode.Topics.Pub.Object.Msg);
        }
        /* USER END: add Objects into MarkerArray*/

        /* USER BEGIN: IMU data */
        auto sync_IMU = &CMNode.Topics.Pub.OdomIMU;

        if ((rv = CMCRJob_DoJob(sync_IMU->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething)
        {
            LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sync_IMU->Job), CMCRJob_RVStr(rv));
        }
        else if (rv == CMCRJob_RV_DoSomething)
        {
            CMNode.Topics.Pub.OdomIMU.Msg.header.frame_id = "Fr0";
            CMNode.Topics.Pub.OdomIMU.Msg.header.stamp = ros::Time(ObjectSensor->TimeStamp);
            if (odom_flag == false)
            {
                x = Car.Fr1.t_0[0];
                y = Car.Fr1.t_0[1];
                z = Car.Fr1.t_0[2];
                odom_flag = true;
            }
            CMNode.Topics.Pub.OdomIMU.Msg.pose.position.x = Car.Fr1.t_0[0] - x;
            CMNode.Topics.Pub.OdomIMU.Msg.pose.position.y = Car.Fr1.t_0[1] - y;
            CMNode.Topics.Pub.OdomIMU.Msg.pose.position.z = Car.Fr1.t_0[2] - z;
            tf2::Quaternion q;
            q.setEulerZYX(Car.Fr1.r_zyx[2], Car.Fr1.r_zyx[1], Car.Fr1.r_zyx[0]);
            CMNode.Topics.Pub.OdomIMU.Msg.pose.orientation.w = q.w();
            CMNode.Topics.Pub.OdomIMU.Msg.pose.orientation.x = q.x();
            CMNode.Topics.Pub.OdomIMU.Msg.pose.orientation.y = q.y();
            CMNode.Topics.Pub.OdomIMU.Msg.pose.orientation.z = q.z();

            tf2::Stamped<tf2::Transform> TF;
            tf2::Quaternion q2;
            q2.setEulerZYX(-3.1415926 / 2, 0, 0);
            TF.setRotation(q2);
            TF.setOrigin(tf2::Vector3(0, 0, 0));
            tf2::Stamped<tf2::Transform> pose;
            geometry_msgs::Pose out;
            tf2::fromMsg(CMNode.Topics.Pub.OdomIMU.Msg, pose);
            tf2::toMsg(TF * pose, out);
            CMNode.Topics.Pub.OdomIMU.Msg.pose = out;
            CMNode.Topics.Pub.OdomIMU.Pub.publish(CMNode.Topics.Pub.OdomIMU.Msg);

            //publish TF
            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.frame_id = "Fr0";
            transformStamped.header.stamp = ros::Time(ObjectSensor->TimeStamp);
            transformStamped.child_frame_id = "Fr1A";
            transformStamped.transform.translation.x = CMNode.Topics.Pub.OdomIMU.Msg.pose.position.x;
            transformStamped.transform.translation.y = CMNode.Topics.Pub.OdomIMU.Msg.pose.position.y;
            transformStamped.transform.translation.z = CMNode.Topics.Pub.OdomIMU.Msg.pose.position.z;

            transformStamped.transform.rotation.w = q.w();
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            br.sendTransform(transformStamped);

            last_x = CMNode.Topics.Pub.OdomIMU.Msg.pose.position.x;
            last_y = CMNode.Topics.Pub.OdomIMU.Msg.pose.position.y;
            last_z = CMNode.Topics.Pub.OdomIMU.Msg.pose.position.z;
        }
        /* USER END: IMU data */

        /* USER BEGIN: Path data */
        // CMNode.Topics.Pub.PathIMU.Msg.header.frame_id = "Fr0";
        // if(ObjectSensor->TimeStamp <= 0.02) CMNode.Topics.Pub.PathIMU.Msg.poses.clear();
        // CMNode.Topics.Pub.PathIMU.Msg.header.stamp = ros::Time(ObjectSensor->TimeStamp);
        // CMNode.Topics.Pub.PathIMU.Msg.poses.push_back(CMNode.Topics.Pub.OdomIMU.Msg);
        /* USER END: Path data */

        /* 2/2/2021 */
        /* USER BEGIN: GNSS data */
        auto sync_GNSS = &CMNode.Topics.Pub.GNSS_out;

        if ((rv = CMCRJob_DoJob(sync_GNSS->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething)
        {
            LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sync_GNSS->Job), CMCRJob_RVStr(rv));
        }
        else if (rv == CMCRJob_RV_DoSomething)
        {
            CMNode.Topics.Pub.GNSS_out.Msg.header.frame_id = "Fr0";
            CMNode.Topics.Pub.GNSS_out.Msg.header.stamp = ros::Time(SimCore.Time);

            CMNode.Topics.Pub.GNSS_out.Msg.latitude = RAD2DEG(GNavSensor.Receiver.UserPosLlhTsa[0]);
            CMNode.Topics.Pub.GNSS_out.Msg.longitude = RAD2DEG(GNavSensor.Receiver.UserPosLlhTsa[1]);
            CMNode.Topics.Pub.GNSS_out.Msg.altitude = GNavSensor.Receiver.UserPosLlhTsa[2];

            CMNode.Topics.Pub.GNSS_out.Msg.pdop = GNavSensor.Receiver.PDOP;
            CMNode.Topics.Pub.GNSS_out.Msg.hdop = GNavSensor.Receiver.HDOP;
            CMNode.Topics.Pub.GNSS_out.Msg.vdop = GNavSensor.Receiver.VDOP;

            // Direction (degrees from north)
            CMNode.Topics.Pub.GNSS_out.Msg.track = heading_cal(Car.Fr1.r_zyx[2]);;
            // Device orientation (units in degrees)
            CMNode.Topics.Pub.GNSS_out.Msg.pitch = RAD2DEG(Vehicle.Pitch);
            CMNode.Topics.Pub.GNSS_out.Msg.roll = RAD2DEG(Vehicle.Roll);
            CMNode.Topics.Pub.GNSS_out.Msg.dip = RAD2DEG(Vehicle.Yaw);

            for(int fi=0;fi<9;fi++){
                CMNode.Topics.Pub.GNSS_out.Msg.position_covariance[fi] = GNavSensor.Receiver.position_covar[fi];
            }
            
            CMNode.Topics.Pub.GNSS_out.Pub.publish(CMNode.Topics.Pub.GNSS_out.Msg);

        }
        /* USER END: GNSS data */
        /* 2/2/2021 */

        /* 2/8/2021 */
        /* USER BEGIN: GNSS data */
        auto sync_vhcl = &CMNode.Topics.Pub.vhcl_out;

        if ((rv = CMCRJob_DoJob(sync_vhcl->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething)
        {
            LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sync_vhcl->Job), CMCRJob_RVStr(rv));
        }
        else if (rv == CMCRJob_RV_DoSomething)
        {
            CMNode.Topics.Pub.vhcl_out.Msg.header.frame_id = "Fr0";
            CMNode.Topics.Pub.vhcl_out.Msg.header.stamp = ros::Time(SimCore.Time);

            CMNode.Topics.Pub.vhcl_out.Msg.velocity = Vehicle.v;
            CMNode.Topics.Pub.vhcl_out.Msg.heading = heading_cal(Car.Fr1.r_zyx[2]);
            CMNode.Topics.Pub.vhcl_out.Msg.pitch = RAD2DEG(Vehicle.Pitch);
            CMNode.Topics.Pub.vhcl_out.Msg.roll = RAD2DEG(Vehicle.Roll);
            
            CMNode.Topics.Pub.vhcl_out.Pub.publish(CMNode.Topics.Pub.vhcl_out.Msg);

        }
        /* USER END: GNSS data */
        /* 2/8/2021 */

        return 1;
    }

    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Called close to end of CarMaker cycle
    * - See "User.c:User_Out()"
    */
    int CMRosIF_CMNode_Out(void)
    {
        ros::WallTime wtime = ros::WallTime::now();

        /* Only do anything if simulation is running */
        if (CMNode.Cfg.Mode == CMNode_Mode_Disabled || SimCore.State != SCState_Simulate)
            return 0;

        int rv;
        auto out = &CMNode.Topics.Pub.CM2Ext;

        /* Communicate to External ROS Node in this cycle?
        * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
        */
        if ((rv = CMCRJob_DoJob(out->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething)
        {
            LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(out->Job), CMCRJob_RVStr(rv));
        }
        else if (rv == CMCRJob_RV_DoSomething)
        {

            out->Msg.cycleno = CMNode.CycleNoRel;
            out->Msg.time = ros::Time(SimCore.Time);
            out->Msg.synthdelay = CMNode.Sync.SynthDelay;

            /* Header stamp and frame needs to be set manually! */

            /* provide system time close to data is sent */
            wtime = ros::WallTime::now();
            out->Msg.header.stamp.sec = wtime.sec;
            out->Msg.header.stamp.nsec = wtime.nsec;

            out->Pub.publish(out->Msg);

            /* Remember cycle for debugging */
            CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
        }

        /* Publish "/clock" topic after all other other topics are published
        * - Is the order of arrival in other node identical? */
        if (CMNode.Cfg.nCyclesClock > 0 && CMNode.CycleNoRel % CMNode.Cfg.nCyclesClock == 0)
        {
            CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(SimCore.Time);
            CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
        }

        /* ToDo: When increase? */
        CMNode.CycleNoRel++;

        return 1;
    }

    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Called one Time when CarMaker ends
    * - See "User.c:User_End()"
    */
    int
    CMRosIF_CMNode_End(void)
    {

        LOG("%s: End", __func__);

        if (ros::isInitialized())
        {

            /* Needs to be called before program exists, otherwise
	 * "boost" error due to shared library and default deconstructor */
            CMNode.Cfg.Ros.Node->shutdown();

            /* ToDo:
	 * - Blocking call? Wait until shutdown has finished?
	 * - Optional? */
            ros::shutdown();
        }

        return 1;
    }

    /*!
    * Important:
    * - NOT automatically called by CMRosIF extension
    *
    * Description:
    * - Example of user generated function
    * - Can be accessed in other sources, e.g. User.c
    * - Use "CMRosIF_GetSymbol()" to get symbol (see "lib/CMRosIF.h")
    *
    */
    int
    CMRosIF_CMNode_MyFunc(char *LogMsg)
    {

        LOG("%s: %s", __func__, LogMsg);
        return 1;
    }

#ifdef __cplusplus
}
#endif
