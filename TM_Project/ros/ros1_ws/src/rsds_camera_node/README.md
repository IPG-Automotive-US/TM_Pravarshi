# CarMaker Raw Signal Data Stream (RSDS) ROS Nodes
This ReadMe provides information on the RSDS ROS nodes that receive standard CarMaker / TruckMaker Camera or Lidar RSI streams, then re-publish the data to standard ROS topics. These nodes are developed from the standard *RSDS Client* examples seen in CarMaker / TruckMaker version 10+.

These nodes were developed to increase runtime efficiency of CMRosIF when using Lidar and Camera RSI sensors, so that publishing of these topics does not need to be done from the integrated CarMaker node (aka CMNode). This allows parallel processing & packaging of data for lidar and camera, while other sensors (GNSS, IMU, etc.) can be published at the same time from CMNode. 

----------------------

## **Quick Start Guide**
Build the ROS workspace with the packages `rsds_lidar_node` and `rsds_camera_node` included in the workspace (use the script `build_cmrosif.sh` on the top level of the project when using CMRosIF).

### **How to Start the RSDS Client Nodes**
Startup scripts are provided for both nodes on the top level of the project:

```
RSDS_Lidar_Start.sh
RSDS_Camera_Start.sh
```

These scripts simply run a `roslaunch` command for the given node, after sourcing the `ros_setup.bash` file:

```
source ros_setup.bash
roslaunch rsds_camera_node SingleCam.launch
```

These nodes need to subscribe to data published by CarMaker GPUSensor processes. These are typically automatically started at TestRun start when the vehicle to be simulated uses RSI sensors. When running in an HPC environment like AWS, these need to be started by the startup script process. In either case, the RSDS nodes need these GPUSensors to be started *before* they can launch and run properly. If the GPUSensor(s) and the RSDS node(s) are started at roughly the same time, this is usually not a problem, as the nodes have a ~10 second timeout to try connecting to a running GPUSensor process for the given sensor type. 

When running the nodes in an office setting, it is recommended to start them in the following way:

1. Build CMRosIF with `build_cmrosif.sh` on the top level of the project.
2. Launch the TruckMaker GUI with `TMStart.sh`. 
3. Launch the ROS workspace and start the TruckMaker application with *Extras --> CMRosIF --> Launch & Start*.
4. Set the simulation speed of CarMaker / TruckMaker to 'paused' so there is ample time to connect the RSDS clients.
5. Load a TestRun and start the simulation.
6. Start either RSDS start script in a new terminal:
   
    ```
    RSDS_Lidar_Start.sh <num_lidar_sensors>
    
    RSDS_Camera_Start.sh
    ```

7. Set the CarMaker / TruckMaker simulation speed back to 'realtime' or another desired simulation speed.  

The nodes give output to the terminal as to whether they had success connecting to the sensor streams or not. The user needs to ensure that the ports CarMaker / TruckMaker sensors are publishing to (see Main GUI --> Parameters --> Sensor Cluster --> 'Socket') match the ports that the RSDS nodes expect. For the Lidar RSDS node, this is parameterized in the launch file with the value of the launch 'param' called `tcp_port` in the launch files `rsds_lidar_node_<num>.launch`:

```xml
<launch>
    <group ns="/rsds_lidar_node_0">
        <param name="tcp_port" type="int" value="2210" />
        <node name="rsds_lidar_node_0" pkg="rsds_lidar_node" type="rsds_lidar_node" />
    </group>
</launch>
```

For the Camera RSDS node, the port is expected to be 2210 and +1 for every extra camera stream thereafter. This can be altered in the launch file `rsds_camera_node.launch`:

```xml
  <arg doc="Connection port"
       name="rsds_port" default="$(eval 2210 + arg('rsds_config'))"/>
```

Alternatively, these can be hard-coded in the C++ code for each RSDS node, but this is not a scalable approach when launching more than one node (i.e. multiple camera or lidar sensors).

Once the RSDS nodes are running, the user will see new topics being published for lidar and camera data:

```
/rsds_lidar_node_0/pointcloud

/front_camera_rgb/image_raw
/front_camera_rgb/...
```

The lidar topic `pointcloud` publishes to the standard ROS message type `sensor_msgs/PointCloud2`, and the camera node publishes to the message type `sensor_msgs/Image`.


### **How to Start Multiple Instances of RSDS Nodes**
#### **Camera**
The top level launch file that is started from `RSDS_Camera_Start.sh` uses `roslaunch` with the launch file `SingleCam.launch`. This launch file is designed for a single camera instance, and it references a second launch file called `rsds_camera_node.launch`. This second launch file contains parameter calculations and does generally not need to be changed. 

To start multiple instances of the RSDS Camera Node, one can edit the file `SingleCam.launch` or copy it and create a second file (then reference if from the top level bash script `RSDS_Camera_Start.sh`). The launch arguments can be copied and pasted for a second instance and change the name of each camera, as well as the other specifics for each camera like resolution and position / orientation.

- Single Camera launch example:

```xml
<launch>

<arg doc="Movie startup delay factor for multiple instances"
     name="delay_factor" default="1.25"/>

<arg doc="Launch rqt with custom perspective"
     name="start_rqt" default="true"/>

<arg doc="Name of Camera 1 for ROS topic"
     name="camera_name_1" default="front_camera_rgb"/>

<arg doc="Custom rqt perspective file"
     name="perspective"
     default="$(find rsds_camera_node)/launch/SingleCam.perspective"/>


<include file="$(find rsds_camera_node)/launch/rsds_camera_node.launch">
  <arg name="camera_name" default="$(arg camera_name_1)"/>
  <arg name="rsds_config" default="0"/>
  <arg name="delay_factor" default="$(arg delay_factor)"/>
  <arg name="param_trans_rot" default="[5, 0, 2.44, 0, 0, 0]"/>
  <arg name="width" default="720"/>
  <arg name="height" default="400"/>
  <arg name="fov_deg" default="90"/>
  <arg name="start_movie" default="false"/>
  <arg name="start_rqt_image_view" default="false"/>
  </include>

</launch>
```

- Multiple Camera launch example:
  
```xml
<launch>

<arg doc="Movie startup delay factor for multiple instances"
     name="delay_factor" default="1.25"/>

<arg doc="Launch rqt with custom perspective"
     name="start_rqt" default="true"/>

<arg doc="Name of Camera 1 for ROS topic"
     name="camera_name_1" default="FrontCamera"/>

<arg doc="Name of Camera 2 for ROS topic"
     name="camera_name_2" default="RearCamera"/>

<arg doc="Custom rqt perspective file"
     name="perspective"
     default="$(find rsds_camera_node)/launch/MultiCam.perspective"/>


<include file="$(find rsds_camera_node)/launch/rsds_camera_node.launch">
  <arg name="camera_name" default="$(arg camera_name_1)"/>
  <arg name="rsds_config" default="0"/>
  <arg name="delay_factor" default="$(arg delay_factor)"/>
  <arg name="param_trans_rot" default="[2.5, 0, 1.3, 0, 0, 0]"/>
  <arg name="width" default="1920"/>
  <arg name="height" default="1080"/>
  <arg name="fov_deg" default="60"/>
  <arg name="start_movie" default="false"/>
  <arg name="start_rqt_image_view" default="false"/>
  </include>

<include file="$(find rsds_camera_node)/launch/rsds_camera_node.launch">
  <arg name="camera_name" default="$(arg camera_name_2)"/>
  <arg name="rsds_config" default="1"/>
  <arg name="delay_factor" default="$(arg delay_factor)"/>
  <arg name="param_trans_rot" default="[1.5, 0, 1.0, 0, 0, 180]"/>
  <arg name="width" default="1920"/>
  <arg name="height" default="1080"/>
  <arg name="fov_deg" default="60"/>
  <arg name="start_movie" default="false"/>
  <arg name="start_rqt_image_view" default="false"/>
  </include>

</launch>

```

#### **Lidar**
The startup bash script for the Lidar RSDS Node allows for an input argument for number of lidar sensors being used in the given simulation. This can be used in the following way:

```bash
RSDS_Lidar_Start.sh 2
```

The above example will start *two* lidar RSDS nodes, each attaching to a different port being published from CarMaker (see previous sections for designating port number). In this case, where two instances are started, it also requires two separate launch files:

```
rsds_lidar_node_0.launch
rsds_lidar_node_1.launch
```

These launch files each use a different namespace and port number, and will launch lidar RSDS nodes in alternate namespaces:

```
/rsds_lidar_node_0/pointcloud
/rsds_lidar_node_1/pointcloud
```

For a single lidar sensor, the start script requires a command line entry of '1', and cannot be left blank.

```bash
RSDS_Lidar_Start.sh 1
```

----------------------

## **Modifications to CMNode in Isuzu Project**
In order to separate the Lidar and Camera publishing from CMNode, Isuzu code that was used to publish lidar and camera data in `CMNode_ROS1_HelloCM.cpp` needed to be commented out (to prevent duplicate publishing). The code was kept as it was received by the IPG team, but just commented out where necessary:

- All code for publishers `Lidar` and `Lidar2`.
- All function calls to `VDS` code, defined in `vds-client.cpp`.
- IPG default logic in CMNode modified for ability to run headless:
  
    ```C++
        if (/*SimCore.CycleNo == 0 ||*/ Inf == NULL || *pmode == CMNode_Mode_Disabled)
        {
            *pmode = CMNode_Mode_Disabled;
            LOG("CarMaker ROS Node is disabled!");
            return 0;
        }
    ```

----------------------

## **Code Overview**
The code for each RSDS client node is developed from the example RSDS client C-code provided in the CarMaker 10 'Examples/RSDS' directory. The code extension is renamed and changed for C++:

- Lidar: `rsds-client.c` ==> `rsds_lidar_node.cpp`
- Camera: `rsds-client-camera-standalone.c` ==> `rsds_camera_node.cpp`

After renaming, the code for Radar and Ultrasonic RSI was eliminated from the standard `rsds-client.c` code so that only relevant Lidar RSI code was kept for `rsds_lidar_node.cpp`.

A direct comparison of the files can be done, but here are the highlights to the modifications to make these clients into ROS nodes:

### **Lidar**
- The CMakeLists.txt file in the package has been modified to use the CarMaker installation 'include' directory, as well as the ROS package's include directory. The node also links with two CarMaker libraries that provide functionality for reading InfoFiles in the C++ code. Some other CarMaker functionality can also be included in this way if needed.

```makefile
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} /opt/ipg/carmaker/linux64-10.0.1/include)

## Add source executable, InfoFile lib(s) path
link_directories(${PROJECT_NAME} /opt/ipg/carmaker/linux64-10.0.1/lib)

find_library(ifile_lib libinfofile.a /opt/ipg/carmaker/linux64-10.0.1/lib/)
find_library(cm_lib libcarmaker.a /opt/ipg/carmaker/linux64-10.0.1/lib/)

add_executable(${PROJECT_NAME} src/rsds_lidar_node.cpp)
```

- CarMaker and ROS header files are included for desired functionality:
```C++
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
```
- Global shared pointer for PointCloud2 message type (so it can be used in `main()` and other functions):
```C++
// Global shared pointer 
sensor_msgs::PointCloud2Ptr pointcloud = boost::make_shared<sensor_msgs::PointCloud2>();
```

- During initialization of the RSDS node:
  - The 'fields' are set for the point cloud:

  ```C++
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

  ...
  ```

  - The beam table of the Lidar RSI sensor being used is via InfoFile Read functions from CarMaker:
  ```C++
  // Add path for Sensor Parameter InfoFile
  path_sensorParam = path_sensorParam + "/Data/Sensor/LidarRSI_pandarQT_M"; //change file name when beam file name changes

  // Read Sensor InfoFile
  int InfoErr = iRead2(&err, Inf_Sensor, path_sensorParam.c_str(), "LidarCode");

  if (InfoErr == 0) {
      
      nBeams = iGetFixedTable2(Inf_Sensor, "Beams.N", 2, 1);
      BeamTable = iGetTableOpt2(Inf_Sensor, "Beams", NULL, 6, &rows);
      
      double *fov_h = iGetFixedTable2(Inf_Sensor, "Beams.FoVH", 2, 1);
      double *fov_v = iGetFixedTable2(Inf_Sensor, "Beams.FoVV", 2, 1);

  ...
  ```

  - The ROS launch parameter `tcp_port` is read in, as well as the current namespace being used:

  ```C++
  // Get TCP port of current namespace
  if (nh.getParam(ros::this_node::getNamespace() + "/tcp_port", RSDScfg.MoviePort) == true) {
      printf("Received ROS Parameter tcp_port: %i\n", RSDScfg.MoviePort);
  } else {
      printf("TCP Port parameter was not read! Please check launch files and sensor configuration!\n");
  };
  ```

- ROS topic is advertised and initialized in normal ROS way:

```C++
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

```

- Lidar RSI data received during simulation steps is packaged into PointCloud2 message within `processLidarData()`:

```C++
void processLidarData(const char *dataP)
{
    ...
    
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
...
}

```
- When Lidar RSI data is received, it is packaged into the message as shown, then published in the `main()` while loop:

```C++
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
```

### **Camera** 
- All references to functions used in the standard RSDS Camera client code that save images to a file are eliminated, as they are not necessary for the ROS node's functionality. 

- ROS header files are included at the beginning of the source file `rsds_camera_node.cpp` (no extra CarMaker header files or libraries used in this node):

```C++
#include "ros/ros.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <angles/angles.h>
```

- Global shared pointer created in the `main()` function, to be used in other functions; ROS node initialized, topic advertised:
```C++
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rsds_camera_node");
    ros::NodeHandle nh("~");

    // shared image message
    sensor_msgs::ImagePtr img_ = boost::make_shared<sensor_msgs::Image>();
    image_transport::CameraPublisher image_pub_;
    sensor_msgs::CameraInfoPtr ci = boost::make_shared<sensor_msgs::CameraInfo>();

    tf2_ros::StaticTransformBroadcaster br;

    // advertise the main image topic
    image_transport::ImageTransport it(nh);
    image_pub_ = it.advertiseCamera("image_raw", 1);
```

- After initializing the RSDS client's connection in `main()`, ROS launch parameters are read in, specific to the camera being simulated:
```C++
...
std::string param_host = "localhost";
nh.param("RSDS_host", RSDScfg.MovieHost, param_host);

nh.param("RSDS_port", RSDScfg.MoviePort, 2211);
nh.param("connection_tries", RSDScfg.ConnectionTries, 10);

std::string camera_frame = "";
nh.param("camera_frame", img_->header.frame_id, camera_frame);

// translation and rotation
std::vector<double> param_trans_rot;
param_trans_rot.assign(6, 0);
nh.param("param_trans_rot", param_trans_rot, param_trans_rot);

geometry_msgs::TransformStamped tf;
tf.header.stamp = ros::Time(0);
tf.header.frame_id = "Fr1A";
tf.child_frame_id = img_->header.frame_id;
tf.transform.translation = tf2::toMsg(tf2::Vector3(param_trans_rot[0], param_trans_rot[1], param_trans_rot[2]));
tf2::Quaternion rot;
tf2::Vector3 default_offset(-90, 0, -90);
rot.setRPY(angles::from_degrees(param_trans_rot[3] + default_offset[0]),
            angles::from_degrees(param_trans_rot[4] + default_offset[1]),
            angles::from_degrees(param_trans_rot[5] + default_offset[2]));
tf.transform.rotation = tf2::toMsg(rot);
br.sendTransform(tf);
...
```

- Package received image data in `RSDS_GetData()`; supported image types are rgb, grey, grey16, and depth16:
```C++
...
const std::string encoding = static_cast<std::string>(ImgType);

if (encoding == "rgb") {
    image->encoding = sensor_msgs::image_encodings::RGB8;
    } else if  (encoding == "grey") {
    image->encoding = sensor_msgs::image_encodings::MONO8;
    } else if  (encoding == "grey16") {
    image->encoding = sensor_msgs::image_encodings::MONO16;
    } else if  (encoding == "depth16") {
    image->encoding = sensor_msgs::image_encodings::MONO16;
    } else {
    ROS_ERROR("Incompatible image type/encoding: %s. Supported output formats: rgb, grey, grey16 and depth16.", ImgType);
    }

image->width = static_cast<uint>(ImgWidth);
image->height = static_cast<uint>(ImgHeight);
image->step = image->width * static_cast<uint>(sensor_msgs::image_encodings::numChannels(image->encoding)
                                        * 0.125 * sensor_msgs::image_encodings::bitDepth(image->encoding));
image->is_bigendian = false;

image->header.stamp = ros::Time(SimTime);
...
```

- Publish image data in `main()` while loop:
```C++
while (ros::ok())
{

    /* Read from TCP/IP-Port and process the image */
    if (RSDS_RecvHdr(RSDScfg.sock,RSDScfg.sbuf) != 0 || RSDScfg.TerminationRequested) {
        break;
    }

    RSDS_GetData(img_);

    // grab the camera info
    ci->header = img_->header;
    ci->height = img_->height;
    ci->width = img_->width;

        // publish the image
    image_pub_.publish(img_, ci);

    ros::spinOnce();
}
``` 