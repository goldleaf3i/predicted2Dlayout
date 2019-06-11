#!/usr/bin/env python

#ros library
import roslib
import rospy
from nav_msgs.msg import OccupancyGrid
from floorplan_analyzer.srv import *
import numpy as np
import calcolateFrontier as cp
import cv2
import pickle
from ros_object import MyGridMap
import numpy as np

class FpTester:
	def __init__(self):		
		self.pub = rospy.Publisher('analyzer',OccupancyGrid,queue_size = 10)
		rospy.init_node('floorplan_tester', anonymous = False)
		self.serveFront = rospy.Service('frontierService', FrontierService, self.serveFront)
		self.floorplan_tester()


	def floorplan_tester(self):
		with open('./src/floorplan_analyzer/src/Map2.txt', 'rb') as fileMap:
    			grid_msg = pickle.load(fileMap)
		myMap = MyGridMap.MyGridMap(grid_msg)
		self.frontier = cp.calculate_frontier(myMap)
		print "Calcolated Frontier"
		self.pub.publish(grid_msg)
		rospy.logdebug("Grid Map sent!")
		rospy.spin()


	def serveFront(self,req):
		indexListSe = []
		lenghtListSe = []
		clusterListSe = []
		count = 0
		for n in self.frontier:
			x=np.uint32(n)
			indexListSe.extend(x)
			lenghtListSe.append(len(x))
			for i in range(len(x)):
				clusterListSe.append(count)
			count = count + 1
				
		#do thing
		numberOfFrontierSe = np.uint8(len(self.frontier))
		return FrontierServiceResponse(indexListSe,clusterListSe,lenghtListSe,numberOfFrontierSe)

if __name__ == '__main__':
	try:		
		a = FpTester()
	except rospy.ROSInterruptException:
		pass
