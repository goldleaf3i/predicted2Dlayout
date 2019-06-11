
from ros_object import MyGridMap
import numpy as np
import pickle
from PIL import Image
import cv2
import time
'''
	Created by Luca Fochetta. Used for testing. Too slow for real application.
'''
def calculate_frontier(grid):
	frontierList = [[]]
	start = time.time()
	mapSize = grid.getSize()
	x = 978
	y = 984
	#startIndex = grid.getIndex(x,y)
	startIndex = x + (grid.getHeight() -y -1)*grid.getWidth()
	while grid.isFree(startIndex) == False:
		print "Searching index"
		x = x - 1
		y = y + 1
		startIndex = grid.getIndex(x,y)
	alreadyChecked = np.full(mapSize, False)
	queue = []
	queue.append(startIndex)
	curPos = 0
	curSize = 1
	alreadyChecked[startIndex] = True
	while curPos < curSize and curPos < mapSize:
		index = queue[curPos]
		if(grid.isFrontier(index)):
			addToFrontierList(grid,index,frontierList)
		ind = np.zeros(4,dtype='int')
		ind[0] = index-1
		ind[1] = index+1
		ind[2] = index + grid.getWidth()
		ind[3] = index - grid.getWidth()
		for i in range(4):
			tempIndex = ind[i]
			if(grid.isFree(tempIndex) and alreadyChecked[tempIndex] == False):
				queue.append(tempIndex)
				curSize = curSize + 1
				alreadyChecked[tempIndex] = True
		curPos = curPos + 1
	print len(frontierList)
	indexToRemove = []
	for i in range(0,len(frontierList)):
		print"Frontier lenght: ", len(frontierList[i])
		if(len(frontierList[i]) == 0):
			indexToRemove.append(i)
	frontierList.pop(i in indexToRemove)
	print len(frontierList)
	return frontierList
	

def addToFrontierList(gridMap,index,frontierList):
	found = False
	nearbyFrontiers = []
	for i in range(len(frontierList)):
		if(isCloseToFrontier(gridMap,frontierList[i],index)):
			found = True
			nearbyFrontiers.append(i)
	if(len(nearbyFrontiers) == 1):
		frontier = nearbyFrontiers[0]
		frontierList[frontier].append(index)
	elif (len(nearbyFrontiers) == 0):
		frontierList.append([index])
		print "New Frontier"
	else:
		"Joining Frontier"
		firstFrontier = nearbyFrontiers[0]
		for j in nearbyFrontiers:
			if((j==firstFrontier)==False):
				print len(frontierList[j]), " ", len(frontierList[firstFrontier])
				frontierList[firstFrontier] = frontierList[firstFrontier] + frontierList[j]
				frontierList.pop(j)
	
def isCloseToFrontier(gridMap,frontier,index):
	for i in range(len(frontier)):
		if(gridMap.areNeighbour(index,frontier[i])):
			return True
	return False

def main():
	with open('Map9.txt','rb') as f:
		x = pickle.load(f)
		m = MyGridMap.MyGridMap(x)
		calculate_frontier(m)

if __name__ == '__main__':
	main()
