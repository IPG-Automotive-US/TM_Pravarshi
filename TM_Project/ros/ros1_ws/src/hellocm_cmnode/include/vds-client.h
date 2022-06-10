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

#ifndef __VDS_CLIENT_H__
#define __VDS_CLIENT_H__

/* ROS */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#ifdef __cplusplus
extern "C" {
#endif

void VDS_Init  (std::vector<ros::Publisher> Pub_in);
void VDS_Start (void);
void VDS_Exit  (void);

#ifdef __cplusplus
}
#endif

#endif /* __VDS_CLIENT_H__ */
