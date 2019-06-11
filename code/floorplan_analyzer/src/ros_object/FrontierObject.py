import MyGridMap

class FrontierObject:
	def __init__(self,isOnline,indexList,clusterList,frontierList,frontierNumber,occ):
		self.isOnline = isOnline
		self.indexList = indexList
		self.clusterList = clusterList
		self.frontierList = frontierList
		self.frontierNumber = frontierNumber
		self.grid = occ
		self.saveMap = False
		
	def hasFrontier(self):
		return self.isOnline
	
	def getGrid(self):
		return self.grid
	
	def getIndexList(self):
		return self.indexList
	def getClusterList(self):
		return self.clusterList
	def getFrontierList(self):
		return self.frontierList
	def getFrontierNumber(self):
		return self.frontierNumber

	def setSaveMap(self,bool):
		self.saveMap = bool

	