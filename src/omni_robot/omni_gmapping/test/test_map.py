#!/usr/bin/env python3
import PIL.Image
import unittest
import subprocess
import sys

import roslib
import os
roslib.load_manifest('gmapping')
import rostest

class TestGmapping(unittest.TestCase):

  # Test that 2 map files are approximately the same
  def cmp_maps(self, f0, f1):
    im0 = PIL.Image.open(f0+'.pgm')
    im1 = PIL.Image.open(f1+'.pgm')
  
    size = 100,100
    im0.thumbnail(size,PIL.Image.ANTIALIAS)
    im1.thumbnail(size,PIL.Image.ANTIALIAS)
  
    # get raw data for comparison
    im0d = im0.getdata()
    im1d = im1.getdata()
  
    # assert len(i0)==len(i1)
    self.assertTrue(len(im0d) == len(im1d))
  
    #compare pixel by pixel for thumbnails
    error_count = 0
    error_total = 0
    pixel_tol = 0
    total_error_tol = 0.1
    for i in range(len(im0d)):
      (p0) = im0d[i]
      (p1) = im1d[i]
      if abs(p0-p1) > pixel_tol:
        error_count = error_count + 1
        error_total = error_total + abs(p0-p1)
    error_avg = float(error_total)/float(len(im0d))
    print '%d / %d = %.6f (%.6f)'%(error_total,len(im0d),error_avg,total_error_tol)
    self.assertTrue(error_avg <= total_error_tol)

  def test_basic_localization_stage(self):
    if sys.argv > 1:
      target_time = float(sys.argv[1])

      import time
      import rospy
      rospy.init_node('test', anonymous=True)
      while(rospy.rostime.get_time() == 0.0):
        print 'Waiting for initial time publication'
        time.sleep(0.1)
      start_time = rospy.rostime.get_time()

      while (rospy.rostime.get_time() - start_time) < target_time:
        print 'Waiting for end time %.6f (current: %.6f)'%(target_time,(rospy.rostime.get_time() - start_time))
        time.sleep(0.1)

    f0 = os.path.join(roslib.packages.get_pkg_dir('gmapping'),'test','basic_localization_stage_groundtruth')
    f1 = os.path.join(roslib.packages.get_pkg_dir('gmapping'),'test','basic_localization_stage_generated')

    cmd = ['rosrun', 'map_server', 'map_saver', 'map:=dynamic_map', '-f', f1]
    self.assertTrue(subprocess.call(cmd) == 0)

    self.cmp_maps(f0,f1)

if __name__ == '__main__':
  rostest.run('gmapping', 'gmapping_slam', TestGmapping, sys.argv)
