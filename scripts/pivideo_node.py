#!/usr/bin/env python

from picamera.array import PiRGBArray
from picamera import PiCamera

from threading import Thread
import sys
import time

#import numpy as np

import cv2

import roslib
import rospy
import std_msgs
import sensor_msgs

from cv_bridge import CvBridge, CvBridgeError

class PiVideoStream:
  # Camera Module V2 Sensor Modes
  # 1: 1920x1080, 16:9, 1/10 <= fps <= 30, partial fov
  # 2: 3280x2464,  4:3, 1/10 <= fps <= 15
  # 3: 3280x2464,  4:3, 1/10 <= fps <= 15
  # 4: 1640x1232,  4:3, 1/10 <= fps <= 40, 2x2 binning
  # 5: 1640x922,  16:9, 1/10 <= fps <= 40, 2x2 binning
  # 6: 1280x720,  16:9,    40 < fps <= 90, partial fov, 2x2 binning
  # 7: 640x480,    4:3,    40 < fps <= 90, partial fov, 2x2 binning
  
  def __init__(self, sensor_mode = 4, resolution=(640,480), framerate=30):
    # init camera and stream
    self.camera = PiCamera(resolution=resolution, framerate=framerate, sensor_mode=sensor_mode)

    self.rawCapture = PiRGBArray(self.camera, size=resolution)
    self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port = True)

    self.frame = None
    self.stopped = False

  def start(self):
    # start thread to read frames from video stream
    t = Thread(target=self.update, args=())
    t.daemon = True
    t.start()
    return self

  def update(self):
    # keep looping infinitely until thread is stopped
    for f in self.stream:
      # grab a frame from the stream and clear the stream in prep
      self.frame = f.array
      self.rawCapture.truncate(0)

      # if the thread indicator variable is set, stop the thread
      # and free camera resources
      if self.stopped:
        self.stream.close()
        self.rawCapture.close()
        self.camera.close()
        return

  def read(self):
    return self.frame

  def stop(self):
    self.stopped = True


def main(args):

  framerate = 15

  vs = PiVideoStream(framerate=framerate)
  vs.start()

  # let camera warm up
  time.sleep(2.0)
  
  bridge = CvBridge()
  
  imgpub = rospy.Publisher("image/compressed", sensor_msgs.msg.CompressedImage, queue_size=1)
  
  rospy.init_node('pivideo_node', anonymous=True)

  rate = rospy.Rate(framerate)

  while not rospy.is_shutdown():
    frame = vs.read()
    #msg = sensor_msgs.CompressedImage()
    #msg.header.stamp = rospy.Time.now()
    #msg.format = "png"
    #msg.data = np.array(cv2.imencode('.png', frame)[1]).tostring()
    
    try:
      msg = bridge.cv2_to_compressed_imgmsg(frame, 'png')
      imgpub.publish(msg)
    except CvBridgeError as e:
      print(e)
      
    rate.sleep()

  vs.stop()
  print "Shutting down ROS Pi Camera module"
  time.sleep(0.1) # wait for thread to exit

if __name__ == '__main__':
  main(sys.argv)

