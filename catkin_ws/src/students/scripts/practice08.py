#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-2
# PRACTICE 8 - COLOR SEGMENTATION
#
# Instructions:
# Complete the code to estimate the position of an object 
# given a colored point cloud using color segmentation.
#

import numpy
import cv2
import ros_numpy
import rospy
import math
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped

NAME = "HERNANDEZ_LUVIANO"

def segment_by_color(img_bgr, points):
    #
    # TODO:
    # - Change color space from RGB to HSV.
    #   Check online documentation for cv2.cvtColor function
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    cv2.imshow("HSV", img_hsv)
    # - Determine the pixels whose color is in the color range of the ball.
    #   Check online documentation for cv2.inRange
    img_bin = cv2.inRange(img_hsv, (28,230,127),(32,255,255))
    cv2.imshow("Binary", img_bin)
    # - Calculate the centroid of all pixels in the given color range (ball position).
    #   Check online documentation for cv2.findNonZero and cv2.mean
    indices=cv2.findNonZero(img_bin)
    [img_x, img_y, a, b]=cv2.mean(indices)
    print([img_x, img_y])
    # - Calculate the centroid of the segmented region in the cartesian space
    #   using the point cloud 'points'. Use numpy array notation to process the point cloud data.
    #   Example: 'points[240,320][1]' gets the 'y' value of the point corresponding to
    #   the pixel in the center of the image.
    [x,y,z,counter]=[0,0,0,0]
    for [[c,r]] in indices:
        xt=points[r,c][0]
        yt=points[r,c][1]
        zt=points[r,c][2]
        if math.isnan(xt) or math.isnan(yt) or math.isnan(zt):
            continue
        [x,y,z,counter]=[x+xt,y+yt,z+zt,counter+1]
    
    x=x/counter if counter>0 else 0
    y=y/counter if counter>0 else 0
    z=z/counter if counter>0 else 0

    # Return a tuple of the form [img_c, img_r, x, y, z] where:
    # [img_c, img_r] is the centroid of the segmented region in image coordinates.
    # [x,y,z] is the centroid of the segmented region in cartesian coordinate. 
    #
    #print(img_bgr[100, 300])
    #print(points[100,300])
    #return [100,100,0,0,0.3]
    
    return [img_x, img_y, x,y,z]

def callback_point_cloud(msg):
    global pub_point
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r = numpy.asarray((rgb_arr >> 16) & 255, dtype=numpy.uint8)
    g = numpy.asarray((rgb_arr >> 8) & 255, dtype=numpy.uint8)
    b = numpy.asarray(rgb_arr & 255, dtype=numpy.uint8)
    img_bgr = cv2.merge((b,g,r))
    [centroid_x, centroid_y, x, y, z] = segment_by_color(img_bgr, arr)
    p = PointStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "kinect_link"
    [p.point.x, p.point.y, p.point.z] = [x,y,z]
    pub_point.publish(p)
    cv2.circle(img_bgr, (int(centroid_x), int(centroid_y)), 20, [0, 255, 0], thickness=3)
    cv2.imshow("Color Segmentation", img_bgr)
    cv2.waitKey(1)

def main():
    global pub_point
    print "PRACTICE 08 - " + NAME
    rospy.init_node("practice08")
    rospy.Subscriber("/kinect/points", PointCloud2, callback_point_cloud)
    pub_point = rospy.Publisher('/detected_object', PointStamped, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

