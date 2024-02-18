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
This script takes as an input odometry measurements from an Odometry ROS topic, and saves the measurements into a CSV file for later processing.
"""
import rospy
from nav_msgs.msg import Odometry
import csv

class Odom2CSV:
  def __init__(self):
      self._debug = rospy.get_param("~debug", False)                        # Print debug messages
      self._odom_topic_name = rospy.get_param("~odom_topic_name", "odom")   # Oodometry topic name
      self._output_file_path = rospy.get_param("~output_file_path", "~/output.csv") # Full path to the output csv file

      # Try to open the file in the 'write' mode
      try:
        self._file = open(self._output_file_path, 'w')
      except OSError:
        rospy.logerr("Could not open file: %s", self._output_file_path)
        exit(0)
      
      # create the csv writer
      self._writer = csv.writer(self._file)

      header = ['time_sec', 'position_x', 'position_y', 'position_z', 'velocity_x', 'velocity_y', 'velocity_z']
      self._writer.writerow(header)


      # Odom Subscriber
      rospy.Subscriber(self._odom_topic_name, Odometry, self.odomCallback, queue_size=10)

  def odomCallback(self, msg):
    # Time stamp in seconds
    t = msg.header.stamp.to_sec()
    print("time in seconds {}".format(t))
    # Position measurements
    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y
    pz = msg.pose.pose.position.z
    # Velocity measurements
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    vz = msg.twist.twist.linear.z

    data = [t, px, py, pz, vx, vy, vz]
    self._writer.writerow(data)
    if(self._debug):
      rospy.loginfo("A data row is written to %s", self._output_file_path)

if __name__ == "__main__":
    rospy.init_node('odom_to_csv_node', anonymous=True)
    rospy.loginfo("odom_to_csv_node is started.")

    obj = Odom2CSV()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down odom_to_csv_node")
        obj._file.close()