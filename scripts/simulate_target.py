#!/usr/bin/env python

"""
MIT License

Copyright (c) 2021 Mohamed Abdelkader, mohamedashraf123@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

"""
This script implements some test cases for trajectory prediction
* Test 1:
*   Send an initial  

* Test 2:
"""
import rospy
from std_srvs.srv import Empty, Trigger, TriggerResponse, SetBool, SetBoolResponse
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class TargetSimulation:
  def __init__(self):
    self._dt = rospy.get_param("~dt", 0.03)                         # Sampling time
    self._traj_time_sec = rospy.get_param("~traj_time_sec", 2.0)    # Trajectory time in seconds

    self._frame_id = rospy.get_param("~frame_id", "map")             # Reference frame for the pose estimates

    self._pose_hist_len = rospy.get_param("~pose_hist_len", 100)

    self._target_path_msg = Path()                                  # History of target poses until current pose
    self._pose_list = list()

    # ----------------------- ROS Publishers------------------------#
    self._target_path_pub = rospy.Publisher("target/path", Path, queue_size=10)
    self._target_odom_pub = rospy.Publisher("target/odom", Odometry, queue_size=10)


    # ----------------------- ROS Services -------------------------#
    rospy.Service('~constant_pos_test', Trigger, self.constantPositionSrv)
    rospy.Service('~linear_traj_test', Trigger, self.linearTrajectorySrv)
    rospy.Service('~circular_traj_test', Trigger, self.circularTrajectorySrv)

  def constantPositionSrv(self, req):
    """
    This test sends a constatnt position of target state over _traj_time_sec
    """

    resp = TriggerResponse()
    if self._dt<=0:
      rospy.logerr("[constantPositionSrv] dt <=0")
      resp.success = False
      resp.message="dt <= 0.0"
      return resp

    rate = rospy.Rate(1.0/self._dt)

    # Reset path msg
    self._pose_list = []
    self._target_path_msg.header.frame_id = self._frame_id

    start_t = (rospy.Time.now()).secs # current time in seconds
    while not rospy.is_shutdown():
      now = (rospy.Time.now()).secs
      if ((now-start_t) > self._traj_time_sec):
        break

      odom_msg = Odometry()
      pose_msg = PoseStamped()

      odom_msg.header.frame_id = self._frame_id
      odom_msg.child_frame_id = "base_link"

      pose_msg.header.frame_id = self._frame_id

      pose_msg.header.stamp = rospy.Time.now()
      pose_msg.pose.position.x = 1.0
      pose_msg.pose.position.y = 1.0
      pose_msg.pose.position.z = 1.0
      pose_msg.pose.orientation.w = 1.0 # zero rotation

      odom_msg.header.stamp = rospy.Time.now()
      odom_msg.pose.pose = pose_msg.pose
      self._target_odom_pub.publish(odom_msg)

      self.appendPoseToPath(pose_msg)
      self._target_path_msg.header.stamp = rospy.Time.now()
      self._target_path_msg.poses = self._pose_list
      self._target_path_pub.publish(self._target_path_msg)

      rate.sleep()
    
    resp.success = True
    resp.message="Simulation of constant position trajectory is done."
    return resp

  def linearTrajectorySrv(self, req):
    """
    This test sends a linear trajectory of target states over _traj_time_sec
    """
    resp = TriggerResponse()
    if self._dt<=0:
      rospy.logerr("[constantPositionSrv] dt <=0")
      resp.success = False
      resp.message="dt <= 0.0"
      return resp

    rate = rospy.Rate(1.0/self._dt)

    # Initial positions
    x = 0.0
    y = 0.0
    z = 1.0
    # Constant velocities
    vx = 1.0
    vy = 1.0
    vz = 1.0

    # Reset path msg
    self._pose_list = []
    self._target_path_msg.header.frame_id = self._frame_id

    start_t = (rospy.Time.now()).secs # current time in seconds
    while not rospy.is_shutdown():
      now = (rospy.Time.now()).secs
      if ((now-start_t) > self._traj_time_sec):
        break

      x = x + self._dt*vx
      y = y + self._dt*vy
      z = z + self._dt*vz

      odom_msg = Odometry()
      pose_msg = PoseStamped()

      odom_msg.header.frame_id = self._frame_id
      odom_msg.child_frame_id = "base_link"

      pose_msg.header.frame_id = self._frame_id

      pose_msg.header.stamp = rospy.Time.now()
      pose_msg.pose.position.x = x
      pose_msg.pose.position.y = y
      pose_msg.pose.position.z = z
      pose_msg.pose.orientation.w = 1.0 # zero rotation

      odom_msg.header.stamp = rospy.Time.now()
      odom_msg.pose.pose = pose_msg.pose
      odom_msg.twist.twist.linear.x = vx
      odom_msg.twist.twist.linear.y = vy
      odom_msg.twist.twist.linear.z = vz
      self._target_odom_pub.publish(odom_msg)

      self.appendPoseToPath(pose_msg)
      self._target_path_msg.header.stamp = rospy.Time.now()
      self._target_path_msg.poses = self._pose_list
      self._target_path_pub.publish(self._target_path_msg)

      rate.sleep()
    
    resp.success = True
    resp.message="Simulation of linear position trajectory is done."
    return resp

  def circularTrajectorySrv(self, req):
    """
    This test simulates a circular trajectory 
    """
    pass

  def appendPoseToPath(self, pose):
    if pose is None:
      rospy.logerr("[appendPoseToPath] pose is None")
      return

    # self._target_path_msg.poses.insert(0, pose)
    self._pose_list.append(pose)
    # self._target_path_msg.poses.append(pose)
    if ( len(self._pose_list) > self._pose_hist_len ):
      self._pose_list.pop(0)

if __name__ == "__main__":
    rospy.init_node('target_simulation_node', anonymous=True)
    rospy.loginfo("target_simulation_node is started.")

    ts = TargetSimulation()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down target_simulation_node")
