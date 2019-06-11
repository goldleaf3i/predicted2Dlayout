#!/usr/bin/env python
import traceback
#ros library
import roslib
import rospy
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

import io
import copy
import floorplan as fp
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger,TriggerResponse
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import pickle

import parameters as par
from ros_object import MyGridMap
from ros_object import FrontierObject as fo
from floorplan_analyzer.srv import *
from std_msgs.msg import String
from floorplan_analyzer.msg import *
class FloorPlanAnalyzer:
	
	def __init__(self):
		#define here the publisher for the frontier
		
		rospy.init_node('FloorPlanNode', anonymous = False)
		rospy.Subscriber("analyzer",OccupancyGrid,self.processMap)
		self.pub = rospy.Publisher('analyzerResult', FrontierPrediction, queue_size=1)
		#self.processOccupancyGrid = False
		self.checkFunctionality = rospy.Service('AnalyzerIsAlive',Trigger,self.checkIfIdle)
		self.saveMapService = rospy.Service('/analyzer/savemap',Trigger,self.switchSaveMap)
		self.counter = 0
		self.path = os.environ['HOME']+"/Desktop/SavedMap/Map"
		self.isIdle = True
		self.saveMap = False #used in order to save a map if the the service is called
		#self.path = "/home/fochetz/Desktop/SavedMap/Map"
		
	
	'''
	def requestOccupancyGrid(self,data):
		self.processOccupancyGrid = True
	'''	
	def checkIfIdle(self,req):
		res = TriggerResponse()
		res.success = self.isIdle
		res.message = ""
		return res

	def switchSaveMap(self,req):
		res = TriggerResponse()
		res.success = True
		res.message = "Next map will be saved"
		self.saveMap = True
		return res

	def processMap(self,data):
		gridMap = MyGridMap.MyGridMap(data)
		image = gridMap.createPGM()
		print "Trying to call for Frontier"
		area_list = []
		frontier_list = []
		SAVE_PATH = []
		
		try:
			beginSaveMap = self.saveMap
			frontierServiceCaller = rospy.ServiceProxy('frontierService', FrontierService)
			resp = frontierServiceCaller()
			rospy.logdebug("Frontier Received")
			self.isIdle = False
			print resp.clusterList
			frontierObject = fo.FrontierObject(True,resp.indexList,resp.clusterList,resp.frontiersLenght,resp.frontierNumber,data)
			frontierObject.setSaveMap(beginSaveMap)
			if par.SAVEMAP or par.DISEGNA or par.SAVELOGGER or beginSaveMap:
				SAVE_FOLDER = self.path+str(self.counter)
				while os.path.exists(SAVE_FOLDER):
					self.counter = self.counter + 1
					SAVE_FOLDER = self.path+str(self.counter)
				os.mkdir(SAVE_FOLDER)
				SAVE_PATH = SAVE_FOLDER + '/'

			logger,area_list,frontier_list,touch_edges,aree_grey_pixel_list = fp.floorplan_analyzer(image,frontierObject,self.counter)
			rospy.loginfo("[Analyzer]"+str(self.counter) + " return " + str(len(area_list)) + " frontier")
			
			if par.SAVEMAP or par.SAVELOGGER or beginSaveMap:
				image.save(SAVE_PATH+"MapPhoto.pgm")
				self.counter=self.counter+1
			if par.SAVELOGGER or beginSaveMap:
				SAVE_LOGGERFILE = SAVE_PATH + 'logger.txt'
				with open(SAVE_LOGGERFILE,'w+') as LOGGERFILE:
					print >> LOGGERFILE, "Log: "
					for part in logger:
						print >> LOGGERFILE, part
			if beginSaveMap == self.saveMap:
				self.saveMap = False
			plt.clf()
			msg = FrontierPrediction()
			msg.frontierPredictedArea = area_list
			msg.frontierNumber = frontier_list
			msg.touchMapEdges = touch_edges
			msg.frontierPredictedAreaGreyPixel = aree_grey_pixel_list
			self.pub.publish(msg)
			self.isIdle = True

			
		except Exception as e:
			rospy.logerr(str(e) + str(traceback.format_exc()))
			msg = FrontierPrediction()
			msg.frontierPredictedArea = []
			msg.frontierNumber = []
			msg.touchMapEdges = []
			msg.frontierPredictedAreaGreyPixel = []
			self.pub.publish(msg)
			self.isIdle = True		
		'''		
		frontierServiceCaller = rospy.ServiceProxy('frontierService', FrontierService)
		resp = frontierServiceCaller()
		rospy.logdebug("Frontier Received")
		self.isIdle = False
		print resp.clusterList
		frontierObject = fo.FrontierObject(True,resp.indexList,resp.clusterList,resp.frontiersLenght,resp.frontierNumber,data)
		if par.SAVEMAP or par.DISEGNA or par.SAVELOGGER:
			SAVE_FOLDER = self.path+str(self.counter)
			while os.path.exists(SAVE_FOLDER):
				self.counter = self.counter + 1
				SAVE_FOLDER = self.path+str(self.counter)
			os.mkdir(SAVE_FOLDER)
			SAVE_PATH = SAVE_FOLDER + '/'
		logger,area_list,frontier_list,touch_edges = fp.floorplan_analyzer(image,frontierObject,self.counter)
		
		if par.SAVEMAP or par.SAVELOGGER:
			#image.save(SAVE_PATH+"MapPhoto.pgm")
			self.counter=self.counter+1
		if par.SAVELOGGER:
			SAVE_LOGGERFILE = SAVE_PATH + 'logger.txt'
			with open(SAVE_LOGGERFILE,'w+') as LOGGERFILE:
				print >> LOGGERFILE, "Log: "
				for part in logger:
					print >> LOGGERFILE, part
			rospy.loginfo(str(self.counter) + " return " + str(len(area_list)) + " frontier")
		plt.clf()
		msg = FrontierPrediction()
		msg.frontierPredictedArea = area_list
		msg.frontierNumber = frontier_list
		msg.touchMapEdges = touch_edges
		self.pub.publish(msg)
		self.isIdle = True
		'''
		
if __name__ == '__main__':
	try:
		floorplanAnalyzer = FloorPlanAnalyzer()
		rospy.spin()
	except rospy.ROSInterruptException:
			pass
