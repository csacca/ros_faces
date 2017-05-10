#!/usr/bin/env python

from threading import Thread
import sys
import time

#import numpy as np
import cv2

import dlib

from imutils import face_utils
import imutils

import roslib
import rospy
import std_msgs

from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge, CvBridgeError

class FrameGrabber:
  def __init__(self):
    self.bridge = CvBridge()
    self.imgsub = rospy.Subscriber("/node_1/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=67108864)
    self.frame = None
    self.frame_num = 0
    self.ready = False

  def callback(self, data):
    try:
      self.frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
      self.frame_num = data.header.seq
    except CvBridgeError as e:
      print e
    else:
      #print("[FrameGrabber %f]: got frame" % time.time())
      self.ready = True

  def read(self):
    self.ready=False
    print self.frame_num
    return self.frame
    #return self.frame.copy()
    


class FaceDetector:
  def __init__(self):
    self.detector = dlib.get_frontal_face_detector()
    self.frame_grabber = FrameGrabber()
    self.stopped = False
    
  def start(self):
    t = Thread(target=self.run, args=())
    t.daemon = True
    t.start()
    return self

  def run(self):
    cv2.namedWindow("Faces")
    while True:
      if (self.frame_grabber.ready):
        frame = self.frame_grabber.read()
        print("[FaceDetector %f]: read frame" % time.time())

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        print("[FaceDetector %f]: gray" % time.time())

        rects = self.detector(gray, 0)
        print("[FaceDetector %f]: detect" % time.time())
      
        for (i, rect) in enumerate(rects):
          (x, y, w, h) = face_utils.rect_to_bb(rect)
          cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
          cv2.putText(frame, "Face #{}".format(i+1), (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Faces", frame)
        cv2.waitKey(25)
        print("[FaceDetector %f]: done" % time.time())
      if self.stopped:
        return
  
  def step(self):
    if (self.frame_grabber.ready):
      frame = self.frame_grabber.read()
      print("[FaceDetector %f]: read frame" % time.time())

      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      print("[FaceDetector %f]: gray" % time.time())

      rects = self.detector(gray, 0)
      print("[FaceDetector %f]: detect" % time.time())
      
      for (i, rect) in enumerate(rects):
        (x, y, w, h) = face_utils.rect_to_bb(rect)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(frame, "Face #{}".format(i+1), (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
      cv2.imshow("Faces", frame)
      cv2.waitKey(25)
      print("[FaceDetector %f]: done" % time.time())
  
  def stop(self):
    self.stopped = True
      
      

def main(args):
  face_detector = FaceDetector()
  
  rospy.init_node('face_detector', anonymous=True)
  time.sleep(1)
  
  #cv2.namedWindow("Faces")
  #cv2.startWindowThread()
  
  #face_detector.start()
  
  #try:
  #  rospy.spin()
  #except KeyboardInterrupt:
  #  print("Shutting down face detector node")
  
  while not rospy.is_shutdown():
    face_detector.step()
    time.sleep(0.5)
  
  #face_detector.stop()
  cv2.destroyAllWindows()
  print("Shutting down face detector node")
  time.sleep(0.5)
  

if __name__ == '__main__':
  main(sys.argv)
