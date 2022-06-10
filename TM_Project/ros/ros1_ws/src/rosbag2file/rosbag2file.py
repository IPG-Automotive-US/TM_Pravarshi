import rospy
import sensor_msgs.msg
import visualization_msgs.msg
import ros_numpy
import numpy as np

class rosbag2file:
    def __init__(self):
        self.count = 0
        self.msg_listener()

    def lidar_callback(self, msg):
        pc = ros_numpy.numpify(msg)
        np_points = np.zeros((pc.shape[0], 4), dtype=np.float32)
        np_points[:, 0] = np.resize(pc['x'], pc.shape[0])
        np_points[:, 1] = np.resize(pc['y'], pc.shape[0])
        np_points[:, 2] = np.resize(pc['z'], pc.shape[0])
        np_points[:, 3] = np.resize(pc['intensity'], pc.shape[0]) / (60*1e7)
        np.save("./output/lidar-%06d.npy" % self.count, np_points)
        self.count += 1

    def GT_callback(self, msg):
        marker_array_ = []
        for marker in msg.markers:
            marker_ = []
            marker_.append(marker.pose.position.x)
            marker_.append(marker.pose.position.y)
            marker_.append(marker.pose.position.z)

            marker_.append(marker.pose.orientation.x)
            marker_.append(marker.pose.orientation.y)
            marker_.append(marker.pose.orientation.z)
            marker_.append(marker.pose.orientation.w)

            marker_.append(marker.scale.x)
            marker_.append(marker.scale.y)
            marker_.append(marker.scale.z)

            marker_.append(marker.color.a)
            marker_.append(marker.color.r)
            marker_.append(marker.color.g)
            marker_.append(marker.color.b)
            marker_array_.append(marker_)
        np.save("./output/gtmarker-%06d.npy" % self.count, marker_array_)

    def rgb_callback(self, msg):
        image = ros_numpy.numpify(msg)
        np.save("./output/rgb-%06d.npy" % self.count, image)

    def depth_callback(self, msg):
        image = ros_numpy.numpify(msg)
        np.save("./output/depth-%06d.npy" % self.count, image)

    def msg_listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/velodyne_points", sensor_msgs.msg.PointCloud2, self.lidar_callback)
        rospy.Subscriber("/IPG_camera0", sensor_msgs.msg.Image, self.rgb_callback)
        rospy.Subscriber("/IPG_camera1", sensor_msgs.msg.Image, self.depth_callback)
        rospy.Subscriber("/ObjectList", visualization_msgs.msg.MarkerArray, self.GT_callback)
        rospy.spin()

if __name__ == '__main__':
    t = rosbag2file()
