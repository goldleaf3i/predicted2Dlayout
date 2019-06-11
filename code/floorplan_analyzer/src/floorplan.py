from __future__ import division
# are all of these import needed?
import datetime as dt
import numpy as np
import util.layout as lay
import util.GrafoTopologico as gtop
import util.transitional_kernels as tk
import util.frontiere as fr
from object import Segmento as sg
from util import pickle_util as pk
#from util import accuracy as ac
from util import layout as lay
from util import disegna as dsg
from util import predizionePlan_geometriche as pgeom
from object import Superficie as fc
from object import Spazio as sp
from object import Plan as plan
from util import MCMC as mcmc
from util import valutazione as val
from util import medial as mdl
#from util import accuracy_parziale as acpar
from util import util_XML as xmlutils
from util import metriche_frontiere as mf
from util import helper as  hp
from shapely.geometry import Point
from shapely.geometry import LineString
import math
from shapely.geometry import Polygon
import parameters as par
import cv2
import matplotlib.pyplot as plt
import glob
import time
import shutil
import copy
import os
import io
from PIL import Image

import datetime as dt
import parameters as par

import random
#ROS OBJECT
from ros_object import FrontierObject as fo
from ros_object import MyGridMap as mgm

def start_analysis(parametri_obj,path_obj,img_rgb,frontierObject):
	logger = []
	logger.append(path_obj.metricMap)
	start_timer = time.time()
	timer_precedente = start_timer
	img_ini = img_rgb.copy() #copy of original map
	#Check if image have fornitures (image_to_clean = True)
	if par.image_to_clean:
		k = 15
		l = 9
		ret,thresh1 = cv2.threshold(img_rgb,parametri_obj.cv2thresh,255,cv2.THRESH_BINARY)
		tmp2 = thresh1.copy()
		noisy_pixel =hp.cluster_img(tmp2,k,l,'after',2)
		img_d = img_rgb.copy()
		img_e = img_rgb.copy()
		for i in noisy_pixel:
			img_d[i[0],i[1]]= [255,0,0]
			img_e[i[0],i[1]] = [255,255,255]
		cv2.imwrite('00_'+str(k)+"_"+str(l)+'_removed.png',img_d)
		cv2.imwrite('00_'+str(k)+"_"+str(l)+'_postprocessing.png',img_e)
		img_to_tresh = img_e
	else:
		img_to_tresh = img_rgb
	ret,thresh1 = cv2.threshold(img_to_tresh,parametri_obj.cv2thresh,255,cv2.THRESH_BINARY)
	if par.PRINT:
		print "Threshold applied"
	#OK Up Here
	#------------------------------------------
	#-------CANNY AND HOUGH TO FIND WALLS------
	#------------------------------------------
	walls , canny = lay.start_canny_ed_hough(thresh1,parametri_obj)
	logger.append("Walls " + str(len(walls)))
	if par.PRINT:
		print "walls: ", len(walls)
	if par.DISEGNA:
		# we draw the map and the output of Canny and Hough 
		dsg.disegna_map(img_rgb,filepath = path_obj.filepath, format='pdf')
		dsg.disegna_canny(canny,filepath = path_obj.filepath, format='pdf')
		dsg.disegna_hough(img_rgb,walls,filepath = path_obj.filepath, format='pdf')
	
	lines = lay.flip_lines(walls, img_rgb.shape[0]-1)
	walls = lay.crea_muri(lines)
	logger.append("Lines " + str(len(lines)))
	if par.PRINT:
		print "lines", len(lines), len(walls)
	if par.DISEGNA:
		# we draw the line that we recovered up to now
		dsg.disegna_segmenti(walls,filepath = path_obj.filepath, format='pdf')
	timer_attuale = time.time()
	logger.append(("Canny,Hough,Walls have been retrieved in: " +str(timer_attuale-timer_precedente) + " seconds"))
	if par.PRINTTIMER:
		print "We applied Canny, hough e identifies the initial candidates for wall line segments in ",timer_attuale-timer_precedente, " seconds"
		
	#------------1.2 SET XMIN YMIN XMAX YMAX for finding  walls-----------------------------------
	#from each candidate points identiy the minimoum and maximom x- and y-axis point as bounding boxes 
	estremi = sg.trova_estremi(walls)
	xmin = estremi[0]
	xmax = estremi[1]
	ymin = estremi[2]
	ymax = estremi[3]
	offset = 20
	#xmin -= offset
	#xmax += offset
	#ymin -= offset
	#ymax += offset

	#-------------------------------------------------------------------------------------
	#---------------1.3_EXTERNAL CONTOUR--------------------------------------------------
	(contours, vertici) = lay.contorno_esterno_versione_tre(img_rgb)
	xmin_contorno = min(vertici)[0]
	xmax_contorno = max(vertici)[0]
	ymin_contorno = min(vertici)[1]
	ymax_contorno = max(vertici)[1]
	xmin = min(xmin,xmin_contorno) - offset
	xmax = max(xmax,xmax_contorno) + offset
	ymin = min(ymin,ymin_contorno) - offset
	ymax = max(ymax,ymax_contorno) + offset
	logger.append("X: " + str(xmin) + " " + str(xmax))
	logger.append("Y: " + str(ymin) + " " + str(ymax))
	if par.DISEGNA:
		dsg.disegna_contorno(vertici,xmin,ymin,xmax,ymax,filepath = path_obj.filepath, format='pdf')
	timer_precedente = timer_attuale
	timer_attuale = time.time()
	logger.append(("Identified the External Contour: "+str(timer_attuale-timer_precedente) + " seconds"))
	if par.PRINTTIMER:
		print "Identified the External Contour ", timer_attuale-timer_precedente
	#-------------------------------------------------------------------------------------
	#---------------1.4_MEAN SHIFT FOR FINDING ANGULAR CLUSTER ---------------------------
	(indici, walls, cluster_angolari) = lay.cluster_ang(parametri_obj.h, parametri_obj.minOffset, walls, diagonali= parametri_obj.diagonali)
	if par.DISEGNA:	
		dsg.disegna_cluster_angolari_corretto(walls, cluster_angolari, filepath = path_obj.filepath,savename = '5b_angular_clusters',format='pdf')
	timer_precedente = timer_attuale
	timer_attuale = time.time()
	logger.append(("Angular Clusters identified in  "+ str(timer_attuale-timer_precedente) + " seconds"))
	if par.PRINTTIMER:
		print "Angular Clusters identified in ", timer_attuale-timer_precedente
	#-------------------------------------------------------------------------------------
	#---------------1.5_SPATIAL CLUSTERS--------------------------------------------------
	# "MURA" -> "WALLS"
	cluster_mura = lay.get_cluster_mura(walls, cluster_angolari, parametri_obj)#metodo di valerio
	#row 125.
	cluster_mura_senza_outliers = []
	for c in cluster_mura:	
		if c!=-1:
			cluster_mura_senza_outliers.append(c)
	# se identify cluster of line segments with the same angular coeeficient (angular clustering) identifying a set of 
	# line segments that could be used for retrieving the walls
	# each cluster is then represented as a single representative line
	segmenti_rappresentanti = lay.get_rappresentanti(walls, cluster_mura_senza_outliers)
	if par.DISEGNA:
		dsg.disegna_segmenti(segmenti_rappresentanti,filepath = path_obj.filepath, savename = "5c_representative_line_segments", format='pdf')
	# classification of segments elected as representative for a specific cluster
	# here we have to set / change the threshold for clustering together the spatial clustering (to cluser together angular clusters that are close)
	segmenti_rappresentanti = sg.spatialClustering(parametri_obj.sogliaLateraleClusterMura, segmenti_rappresentanti)
	# associate all other line segments to the same saptial cluster identified in the previous step 
	cluster_spaziali = lay.new_cluster_spaziale(walls, segmenti_rappresentanti, parametri_obj)
	if par.DISEGNA:
		dsg.disegna_cluster_spaziali(cluster_spaziali, walls,filepath = path_obj.filepath, format='pdf')
		dsg.disegna_cluster_mura(cluster_mura, walls,filepath = path_obj.filepath, savename= '5d_cluster_walls', format='pdf')
	timer_precedente = timer_attuale
	timer_attuale = time.time()
	logger.append(("Spatial clusters, and representative line segments computed in: "+str(timer_attuale-timer_precedente) + " seconds"))
	if par.PRINTTIMER:
		print "Spatial clusters, and representative line segments computed in: ", timer_attuale - timer_precedente, " seconds"
	#-------------------------------------------------------------------------------------
	#-------------------1.6_CREATION REPRESENTATIVE EXTENDED_LINES------------------------
	(extended_lines, extended_segments) = lay.extend_line(cluster_spaziali, walls, xmin, xmax, ymin, ymax,filepath = path_obj.filepath)
	if par.DISEGNA:
		dsg.disegna_extended_segments(extended_segments, walls,filepath = path_obj.filepath, format='pdf')	
	#-------------------------------------------------------------------------------------
	#-------------1.7_CREO GLI EDGES TRAMITE INTERSEZIONI TRA EXTENDED_LINES--------------
	edges = sg.crea_edges(extended_segments)
	#-------------------------------------------------------------------------------------
	#----------------------1.8_SETTO PESI DEGLI EDGES-------------------------------------
	edges = sg.setPeso(edges, walls)
	#-------------------------------------------------------------------------------------
	#----------------1.9_CREO LE CELLE DAGLI EDGES----------------------------------------
	celle = fc.crea_celle(edges)
	timer_precedente = timer_attuale
	timer_attuale = time.time()
	logger.append(("Creation of Edges,Weight edges, and cells took: "+ str(timer_attuale-timer_precedente) + " seconds"))
	if par.PRINTTIMER:
		print "Creation of Edges,Weight edges, and cells took: " , timer_attuale - timer_precedente, " seconds"
	#-------------------------------------------------------------------------------------
	#----------------CLASSIFICATION OF CELLS----------------------------------------------
	# HERE I IDENTIFY INTERNAL AND EXTERNAL CELLS -> PARTIAL CELLS ARE CLASSIFIED AND IDENTIFIED LATER -AFTER LAYOUT RECONSTRUCTION
	global centroid
	# WE HAVE ANOTHER METHOD FOR CLASSIFICATION OF CELLS IS CALLED classifica_celle_con_percentuale (USES A PERCENTAGE OF CELLS)
	(celle, celle_out, celle_poligoni, indici, celle_parziali, contorno, centroid, punti) = lay.classificazione_superfici(vertici, celle)
	#-------------------------------------------------------------------------------------
	timer_precedente = timer_attuale
	timer_attuale = time.time()
	logger.append("Classification of Cells into Fully or Not Explored took: " + str(timer_attuale-timer_precedente))
	if par.PRINTTIMER:
		print "Classification of Cells into Fully or Not Explored took:", timer_attuale - timer_precedente, " seconds"
	#OK UP HERE. ORIGINAL ROW 193
	#--------------------------POLYGON FROM CELLS-----------------------------------------	
	(celle_poligoni, out_poligoni, parz_poligoni, centroid) = lay.crea_poligoni_da_celle(celle, celle_out, celle_parziali)
	if par.DISEGNA:
		dsg.disegna_poligoni_interni_esterni(celle_poligoni, out_poligoni, parz_poligoni, xmin, ymin, xmax, ymax, format='pdf', filepath = path_obj.filepath, savename = '8_d_celle_in_out')

	#-------------------------------------------------------------------------------------
	#------------------CREATION OF MATRICES L, D, D^-1, ED M = D^-1 * L-------------------
	(matrice_l, matrice_d, matrice_d_inv, X) = lay.crea_matrici(celle, sigma = parametri_obj.sigma)
	#-------------------------------------------------------------------------------------
	#----------------DBSCAN FOR CLUSTERING NEARBY CELLS INTO ROOMS -----------------------
	clustersCelle = lay.DB_scan(parametri_obj.eps, parametri_obj.minPts, X, celle_poligoni)
	#-------------------------------------------------------------------------------------
	colori = dsg.get_colors(clustersCelle)
	if par.DISEGNA:
		dsg.disegna_dbscan(clustersCelle, celle, celle_poligoni, xmin, ymin, xmax, ymax, edges, contours,filepath = path_obj.filepath, format='pdf')
	timer_precedente = timer_attuale
	timer_attuale = time.time()
	logger.append(("DBSCAN in: "+str(timer_attuale-timer_precedente) + " seconds"))
	if par.PRINTTIMER:
		print "DBSCAN took ", timer_attuale - timer_precedente, " seconds"
	#-------------------------------------------------------------------------------------
	#------------------MERGE CELLS INTO ROOM POLYGONS-------------------------------------
	stanze, spazi = lay.crea_spazio(clustersCelle, celle, celle_poligoni, colori, xmin, ymin, xmax, ymax, filepath = path_obj.filepath) 
	if par.DISEGNA:
		dsg.disegna_stanze(stanze, colori, xmin, ymin, xmax, ymax,filepath = path_obj.filepath, format='pdf')
	#-------------------------------------------------------------------------------------
	#### END OF THE METHOD FOR LAYOUT RECONSTUCTION OF FULLY EXPLORED ROOMS 

	#### START OF THE METHOD FOR PREDICTING THE SHAPE OF PARTIALLY ROOMS IN THE MAP
	

	# RETRIEVING PARTIAL CELLS - THIS SHOULD BE IMPROVED AS WE ARE EXCLUDING ROOMS WITH SMALL FRONTIERS
	coordinate_bordi = [xmin, ymin, xmax, ymax]
	celle_parziali, parz_poligoni = lay.get_celle_parziali(celle, celle_out, coordinate_bordi) # REFACTOR: CHECK THIS PROBABLY COULD BE REMOVED
	# IDENTIFICATION OF POLYGONS REPRESENTING CELLS OUTSIDE THE KNOWN MAP (FOR EXPANDING ROOMS)
	out_poligoni = lay.get_poligoni_out(celle_out)
	if par.DISEGNA:
		dsg.disegno_mura_segmenti_stanze(stanze, colori, xmin, ymin, xmax, ymax, cluster_spaziali, walls, format='pdf', filepath = path_obj.filepath, savename = '14_tutto')
		dsg.disegna_pareti(edges, format='pdf', filepath =path_obj.filepath, savename = '14_walls')
	
	#--------END OF LAYOUT RECONSTRUCTION AND PRELIMINARY COMPUTATION FOR PREDICTION-------
	#---------------------------identify partial cells aka "cellette"----------------------
	# create the object PLAN
	# transform the object "polygon" into "cell" object --> "cellette"
	cellette_out = []
	for p,c in zip(out_poligoni, celle_out):	
		celletta = sp.Celletta(p,c)
		celletta.set_celletta_out(True)
		cellette_out.append(celletta)
	#spaces = rooms = "spazi"; contour = object Polygon, cellette_out = list of cells
	# PLAN e' l'oggetto principale	
	plan_o = plan.Plan(spazi, contorno, cellette_out) 
	if par.DISEGNA:
		dsg.disegna_spazi(spazi, colori, xmin, ymin, xmax, ymax,filepath = path_obj.filepath, savename = '13_spaces', format='pdf')
	#preprocessing of layout(reducing undersegmentation and oversegmentation)
	# REFACTOR: THIS STEP SHOULD BE DEFINITIVELY IMPROVED TO IMPROVE LAYOUT RECONSTRUCTION IN PARTIAL MAPS
	timer_precedente = timer_attuale
	timer_attuale = time.time()
	logger.append((" Identification of Partial Cells and preprocessing took: "+str(timer_attuale-timer_precedente)+" seconds"))		
	if par.PRINTTIMER:
		print "Identification of Partial Cells and preprocessing took:", timer_attuale-timer_precedente, " seconds"
	if par.fixUnderandOversegmentazione:
		#-------retrieval of doors - fixing undersegmentation using VORONOI TASSELLATION------
		# draw the doors
		distanceMap, points, b3, b4, critical_points = mdl.critical_points(path_obj.metricMap)
		# b4 are points on medial axis that could be doors, and are put into the list of  critical_points
		if par.DISEGNA:
			dsg.disegna_distance_transform(distanceMap, filepath = path_obj.filepath, format='pdf')
			dsg.disegna_medial_axis(points, b3, filepath = path_obj.filepath, format='pdf')
			dsg.disegna_medial_axis(points, b4, filepath = path_obj.filepath, format='pdf', savename='12_voronoi_doors')
		
		#-------------------------------------------------------------------------------------
		#-----------------------------divide undersegmented spaces ---------------------------
		if par.divideUndeSegmentedRooms:
			plan.separaUndersegmentazione(plan_o, critical_points, parametri_obj, path_obj, xmin, ymin, xmax, ymax)
			if par.DISEGNA:
				dsg.disegna_spazi(plan_o.spazi, dsg.get_colors(plan_o.spazi), xmin, ymin, xmax, ymax,filepath = path_obj.filepath, savename = '13a_UNDERSEGMETAZIONE', format='pdf')
		#-------------------------------------------------------------------------------------
		#---------------------------merge spaces oversegmented  ------------------------------
		
		if par.mergeOverSegmentedRooms:	
			plan.unisciOversegmentazione(plan_o, parametri_obj)
		
		if par.DISEGNA:
			dsg.disegna_spazi(plan_o.spazi, dsg.get_colors(plan_o.spazi), xmin, ymin, xmax, ymax,filepath = path_obj.filepath, savename = '13b_OVERSEGMENTAZIONE', format='pdf')
		timer_precedente = timer_attuale
		timer_attuale = time.time()
		logger.append(("under-Oversegmetazione adjustment in: "+ str(timer_attuale-timer_precedente) + " seconds"))
		if par.PRINTTIMER:			
			print "under-Oversegmetazione adjustment in: ", timer_attuale - timer_precedente, " seconds"
	#-------------------------------------------------------------------------------------	
	#OK up Here.
	# identification of partial cells based on the presence of FRONTIERS in those cells
	#--------------------------------
	#VARIABLE USED ONLY ONLINE
	redList = []
	greenList = []
	blueList = []
	clusterListPerSpazi = []
	#--------------------------------
	timer_attuale = time.time()
	if frontierObject.isOnline == False:	
		immagine_cluster, frontiere, labels, lista_pixel_frontiere = fr.ottieni_frontire_principali(img_ini)
		if len(labels) > 0:
			plan.trova_spazi_parziali_da_frontiere(plan_o, lista_pixel_frontiere, immagine_cluster, labels)
	else:
		redList = []
		greenList = []
		blueList = []
		clusterListPerSpazi = []
		print "Frontier number: ", frontierObject.getFrontierNumber()
		for j in range(frontierObject.getFrontierNumber()):
			redList.append(int(random.random()*255))
			greenList.append(int(random.random()*255))
			blueList.append(int(random.random()*255))
		#diff_color = fr.get_colori_rgb(n_of_color)
		oMap = frontierObject.getGrid()
		gMap = mgm.MyGridMap(oMap)
		lista_pixel_frontiere = []
		labels = []
		counter = 0
		indexList = frontierObject.getIndexList()
		clusterList = frontierObject.getClusterList()
		print clusterList
		immagine_cluster = img_ini.copy()
		for j in range(frontierObject.getFrontierNumber()):
			length = frontierObject.getFrontierList()[j]
			r = redList[j]
			g = greenList[j]			
			b = blueList[j]
			clu = clusterList[j]
			clusterListPerSpazi.append(clu)
			print "r: ",r," g: ",g," b: ",b
			for i in range (length):
				x,y = gMap.getImageCoordinates(indexList[counter+i])		
				tPixel = fr.pixel(r,g,b,x,y)
				tPixel.set_cluster(clu)
				lista_pixel_frontiere.append(tPixel)
				labels.append(clu)
				immagine_cluster[y][x][0] = r
				immagine_cluster[y][x][1] = g
				immagine_cluster[y][x][2] = b
			counter = counter + length

		if len(labels) > 0:
			plan.trova_spazi_parziali_da_frontiere_online(plan_o,redList,greenList,blueList,clusterListPerSpazi,immagine_cluster)
			plan.assegna_area_predetta_celle_parziali(plan_o,redList,greenList,blueList,clusterListPerSpazi,immagine_cluster)

	l_max = -1
	for j in labels:
		if l_max < j :
			l_max = j
	print "Frontiers: ", l_max
	if len(labels) > 0:
		spazi = sp.trova_spazi_parziali_conta_pixel(plan_o.spazi)
	timer_precedente = timer_attuale
	timer_attuale = time.time()
	logger.append(("Frontiers retrieved  in: "+ str(timer_attuale-timer_precedente)+ " seconds"))
	if par.PRINTTIMER:
			print "Frontiers retrieved in : ", timer_attuale-timer_precedente
	if par.DISEGNA:
		dsg.disegna_map(immagine_cluster,filepath = path_obj.filepath, savename = '0a_frontiers', format='png')
		dsg.disegna_spazi(plan_o.spazi, dsg.get_colors(plan_o.spazi), xmin, ymin, xmax, ymax,filepath = path_obj.filepath, savename = '13c_spazi_parziali', format='pdf')
			
	#-------------------END OF SEARCH FOR PARTIAL CELLS-----------------------------------	
	#----------------------------Compute weight forextended_segments----------------------
	

	extended_segments = sg.setPeso(extended_segments, walls)# TODO: check it works
	#compute, for each extended segment how many rooms it passes by (it "covers")
	lay.calcola_copertura_extended_segment(extended_segments, plan_o.spazi)
	plan_o.set_extended_segments(extended_segments)
	#-------------------------------------------------------------------------------------	
	#copy of the initial list of spaces 
	copia_spazi=copy.deepcopy(plan_o.spazi)
	#------------------------------GEOMETRICAL PREDICTION---------------------------------	
	
	# HERE IT BEGINTS THE ACTUAL COMPUTATION OF THE PREDICTED SHAPE 
	# find a list of partial cells 
	cellette_out = plan_o.cellette_esterne
	spazi_parziali = []
	
	# set of the portion of area alraeady explored and identified in the layout. This is the starting area for prediction
	for s in plan_o.spazi:
		if s.parziale == True:
			spazi_parziali.append(s)
			area_predetta_lay = s.spazio.area -(s.spazio.intersection(contorno)).area
			area_originale_stanza = (s.spazio.intersection(contorno)).area
			s.set_area_originale(area_originale_stanza)
			s.set_area_predetta(s.area_predetta+area_predetta_lay)
			if par.PRINT:
				print "Area predicted: ",s.area_predetta
				print "Area before prediction: ", s.area_originale
			logger.append("Area predicted for each space "+str(s.cluster_frontiera)+ ": " + str(s.area_predetta))
	plan_o_2 = copy.deepcopy(plan_o)
	timer_precedente = time.time()
	
	# REFACTOR: I'VE ELIMINATED HERE UNNECESSARY "PREDICTION" OLD STEPS

	
	#-----ADDING TO PARTIAL CELLS THOSE "OUT" CELLS TOUCHED BY AT LEAST A LASER BEAM
	start_time_azione_1 = time.time()
	i=0
	nuovi_spazi = []
	for s in spazi_parziali:
	
		celle_confinanti = pgeom.estrai_celle_confinanti_alle_parziali(plan_o, s)#identifying cells that are touching fully-explored cells belonging to partial rooms.
		# merge cells if touched by a beam of the LIDAR
		celle_confinanti = plan.trova_celle_toccate_dal_laser_beam(celle_confinanti, immagine_cluster)
		logger.append("Cells to be added for prediction in first step are "+ str(len(set(celle_confinanti))))
		# i do not want to add cells that are "behind" a wall of the partial room that i've already perceived 
		celle_confinanti, celle_elimina_pareti = pgeom.elimina_celle_con_parete_vista(celle_confinanti, s)
		logger.append("Cells that i do not add in prediction cause are behind a door " + str(len(celle_elimina_pareti)))
		if len(celle_confinanti)>0:
			# Merge of cells into the SPACE
			area_aggiunta = 0
			for cella in celle_confinanti:
				if cella.vedo_frontiera == True:
					sp.aggiungi_cella_a_spazio(s, cella, plan_o)
					area_aggiunta += cella.cella.area
			# assign to the room the area added after this step
			s.set_area_predetta(s.area_predetta+area_aggiunta)
					

	#--------------------------------
	# CREATION OF NEW ROOMS
	celle_candidate_nuovi_spazi = []
	for s in plan_o.spazi:
		celle_confinanti = pgeom.estrai_celle_esterne_confinanti(plan_o,s)
		if celle_confinanti > 0 and frontierObject.isOnline:
			celle_confinanti = plan.assegna_cluster_frontiera_a_lista_celle(celle_confinanti,redList,greenList,blueList,clusterListPerSpazi,immagine_cluster)
			celle_confinanti = set(celle_confinanti)
			for c in celle_confinanti:
				if c.parziale:
					if c.cluster_frontiera != s.cluster_frontiera:
						celle_candidate_nuovi_spazi.append(c)
					else:
						# ADDING CELLS WHERE THE SAME FRONTIER IS
						sp.aggiungi_cella_a_spazio(s, c, plan_o)
						s.set_area_predetta(s.area_predetta + c.cella.area)
						while celle_candidate_nuovi_spazi.count(c) > 0:
							celle_candidate_nuovi_spazi.remove(c)
	ids=[]
	if par.DISEGNA:
		try:
			dsg.disegna_spazi(plan_o.spazi, dsg.get_colors(plan_o.spazi), xmin, ymin, xmax, ymax,filepath = path_obj.filepath, savename = '14_PREDICTION_STEP_1', format='png')
	#------------------------------------------------------------
		except:
			logger.append("Errore while drawing 14_PREDICTION_STEP_1")
	celle_candidate_nuovi_spazi = set(celle_candidate_nuovi_spazi)
	
	if len(celle_candidate_nuovi_spazi) > 0:
		for spazio in plan_o.spazi:
			ids.append(spazio.id)
		max_id = np.amax(ids)
		logger.append("ADDING "+ str(len(celle_candidate_nuovi_spazi)) + " ROONS")
	for c in celle_candidate_nuovi_spazi:
		if c.out:
			cella_poli = [c.cella]
			cella_corrisp = [c.c]
			stanza = copy.deepcopy(c.cella)
			max_id = max_id + 1
			new_spazio = sp.Spazio(cella_poli,stanza,cella_corrisp,max_id)
			new_spazio.set_out(False)
			new_spazio.set_parziale(True)
			new_spazio.set_area_predetta(new_spazio.area_predetta+(new_spazio.spazio.area-(new_spazio.spazio.intersection(contorno)).area))
			for i in new_spazio.cells: #list of only one element
				i.set_grey_pixel_count(c.grey_pixel_count)
			new_spazio.set_cluster_frontiera(c.cluster_frontiera)
			trovato = False
			bordi = c.c.bordi
			for b in bordi:
				if b.cluster_spaziale == 'bordo1' or b.cluster_spaziale == 'bordo2' or b.cluster_spaziale == 'bordo3' or b.cluster_spaziale == 'bordo4': 
					new_spazio.set_tocco_bordo(True)
					trovato = True
			if trovato==False:
				new_spazio.set_tocco_bordo(False)
			if new_spazio.spazio.area >0:
				nuovi_spazi.append(new_spazio)
				spazi_parziali.append(new_spazio)
				plan_o.spazi.append(new_spazio)
				plan_o.cellette_esterne.remove(c)
			logger.append("ADDED space with cluster " + str(new_spazio.cluster_frontiera) + " and predicted area" + str(new_spazio.area_predetta))
			logger.append("-----------------------------------------------------")

	end_time_azione_1 = time.time()
	if par.DISEGNA:
		dsg.disegna_sovrapposizione_predizione(copia_spazi, plan_o.spazi, contours , dsg.get_colors(plan_o.spazi), xmin, ymin, xmax, ymax, filepath = path_obj.filepath,savename ='14_azione_geom_1_stanze_aggiunte', format='png')
	#-----------------------------MAIN PREDICTION ----------------------------------
	#--- CREATE DATA STRUCTURE 
	lista_n_elementi = [] #number of cells that are close to the predicted room
	lista_n_disposizioni_totali = [] #number of possible permutations of set of cells for each room
	lista_n_disposizioni_reali = []  #number of possible permutations of set of cells for each room which does not violate any constraint
	lista_tempi = [] # time required for predicting a room
	#---

	start_time_azione_complessa = time.time()
	for index,s in enumerate(spazi_parziali):
		start_time_stanza= time.time()
		
		# remove cells from other rooms
		celle_di_altre_stanze = plan.estrai_celle_di_altre_stanze(s,plan_o)
								
		# create the list of possible moves for prediction 
		level= 1 # number of hops used for prediction 
		
		elementi = pgeom.estrai_spazio_delle_celle(s, plan_o, level)
		#elementi = pgeom.elimina_spazi_sul_bordo_da_candidati(elementi, plan_o) # do not consider cells that touch the border
		lista_n_elementi.append(len(elementi))
		logger.append("Elements for predicition are: "+str(len(elementi)))
		if len(elementi) > 9:
			level = 0
			elementi = pgeom.estrai_spazio_delle_celle(s, plan_o, level)
			#elementi = pgeom.elimina_spazi_sul_bordo_da_candidati(elementi, plan_o) # do not consider cells that touch the border
			logger.append("Computing numer of possible elements after going down one level: " +str(len(elementi)))
		if par.PRINT:
			print "elements are ", len(elementi)
			print "-------start counting possible combinations of cells -------"
		permutazioni = pgeom.possibili_combinazioni(elementi)
		lista_n_disposizioni_totali.append(len(permutazioni))
		if par.PRINT:
			print "-------end processing possible combination of cells --------"
			print "possile combinations are ", len(permutazioni)
		
		permutazioni_corrette = []
		if len(permutazioni)>0:
			# for each possible combination, compute the cost that have adding all those cells to the partial rooms
			score_permutazioni_corrette = []
			for indice,permutazione in enumerate(permutazioni):
				ok=False
			
				pgeom.aggiunge_celle_permutazione(permutazione, plan_o, s) #adding cells of this combination to the room
			
				#computing score
				score2_dopo = val.score2(s, plan_o, celle_di_altre_stanze)
				#computing costs (penalties)
				penal1_dopo = val.penalita1(s)# the higher this cost, the more convex the shape of the predicted room is.
				penal4_dopo = val.penalita4(s, plan_o, celle_di_altre_stanze)#number of small segments from extended lines -  to reduce rooms with many "steps"
			
				# if we have a room that is split into more than 1 polygon - prediction is invalid 
				if type(s.spazio)== Polygon:
					ok = True
					permutazioni_corrette.append(permutazione)
					
					# remove duplicate permutation of rooms
					for p in permutazioni:
						vuoto= list(set(p)-set(permutazione))
						if len(vuoto)==0 and len(p)== len(permutazione) and p!= permutazione:
							permutazioni.remove(p)
					
					#------------evaluate the layout with the candidate set of cells for prediction  ---------------
			
					score = val.score_function(score2_dopo, penal1_dopo, penal4_dopo)  #REFACTOR: CHECK IF THIS IS FULLY IMPLEMENTED
					score_permutazioni_corrette.append(score)
			
					#----------------------end of evaluation of prediction -----------------------------------------
				
				else:
					# delete combination of cells taht are not valid
					permutazioni.remove(permutazione)
			
				#------			
				pgeom.elimina_celle_permutazione(permutazione, plan_o, s)
				if ok ==True:
					a=0
				#------	
			
			
			# identify the combination of cells with the highest value for prediction 
			if len(score_permutazioni_corrette)>0:
				max_score = np.amax(score_permutazioni_corrette)#ADDED
				logger.append("Max score: " + str(max_score))
			  	if par.PRINT:
					print "max_score", max_score
				posizione_permutazione = score_permutazioni_corrette.index(max_score)#ADDED
				permutazione_migliore = permutazioni_corrette[posizione_permutazione]
		
				# compare the layout before and after prediction (if adding cells reduces the score)
				# computing the score of layout without prediction 
				score2_prima = val.score2(s, plan_o, celle_di_altre_stanze)
				try:
					penal1_prima = val.penalita1(s)# reduce convex rooms	
				except:
					penal1_prima = 10000
				penal4_prima = val.penalita4(s, plan_o, celle_di_altre_stanze)# reduce steps
				score_originale = val.score_function(score2_prima, penal1_prima, penal4_prima) #REFACTOR: WE SHOUD EVALUATE SUCH PARAMS
				logger.append("Score originale: "+str(score_originale))
			  	if par.PRINT:
					print "score_originale", score_originale
		
				if max_score >= score_originale:
					# PREDICTION IMPROVES SCORE
					permutazione_migliore = permutazione_migliore
					# COMPUTE AREA OF PREDICTION
					area_predetta = 0
					for el in permutazione_migliore:
						area_predetta += el.cella.area
						el.set_grey_pixel_count(plan.calcola_grey_pixel_celletta(el,immagine_cluster))
					# ASSIGN BEST PREDICTION TO ROOM
					s.set_area_predetta(s.area_predetta+area_predetta)
					# ADD CELLS TO PREDICTED ROOM
					pgeom.aggiunge_celle_permutazione(permutazione_migliore, plan_o, s)
				else:
					# NO PREDICTION IS BETTER THAN ANY PREDICTION - DO NOTHING
					pass
			else:
				# NO POSSIBLE PREDICTION - DO NOTHING
				pass
			
		end_time_stanza = time.time()
		lista_tempi.append(end_time_stanza - start_time_stanza) 
		lista_n_disposizioni_reali.append(len(permutazioni_corrette))
		logger.append("PREDICTION OF ROOM "+str(s.cluster_frontiera) +" TOOK "+str(end_time_stanza - start_time_stanza) + " seconds")
		logger.append("-------------------")
		#--------------------------- END PREDICTION-----------------------------		
	end_time_azione_complessa = time.time()
	logger.append("Prediction took: " + str(end_time_azione_complessa - start_time_azione_complessa))
	if par.DISEGNA:
		try:
			dsg.disegna_spazi(plan_o.spazi, dsg.get_colors(plan_o.spazi), xmin, ymin, xmax, ymax,filepath = path_obj.filepath, savename = '15_prediction', format='png')
			## WALLS ARE PARTICULARLY IMPORTANT; WE SHOULD USE THEM 
			dsg.disegna_spazi_con_pareti_nascoste(plan_o.spazi, dsg.get_colors(plan_o.spazi), xmin, ymin, xmax, ymax,edges,filepath = path_obj.filepath, savename = '15_prediction_and_walls', format='pdf')
		except:
			logger.append("ERROR WHILE DRAWING PREDICTION")
	
	if par.PRINTTIMER:
		timer_precedente = timer_attuale
		timer_attuale = time.time()
		logger.append(("PREDICTION "+ par.PREDIZIONE +": ", timer_attuale-timer_precedente))
		print "PREDICTION ",par.PREDIZIONE, " TOOK ", timer_attuale-timer_precedente, " SECONDS"
	# if the predicted area touches the contour, insert a "red border"
	for spazio in plan_o.spazi:
		if spazio.parziale == True:	
			trovato = False
			for cellette in spazio.cells:
				bordi = cellette.c.bordi
				for b in bordi:
					if b.cluster_spaziale == 'bordo1' or b.cluster_spaziale == 'bordo2' or b.cluster_spaziale == 'bordo3' or b.cluster_spaziale == 'bordo4': 
						spazio.set_tocco_bordo(True)
						trovato = True
			if trovato==False:
				spazio.set_tocco_bordo(False)
		else:
			spazio.set_tocco_bordo(False)
	if par.DISEGNA:
		# DRAW PARTIAL MAPS + PREDICTION + BORDER
		dsg.disegna_sovrapposizione_predizione(copia_spazi, plan_o.spazi, contours , dsg.get_colors(plan_o.spazi), xmin, ymin, xmax, ymax, filepath = path_obj.filepath, format='pdf')

	# COMPUTE ACCURACY OF PARTIAL ROOMS - ICRA 19 VERSION 
	stanze_complete = [] #ONLY FULLY EXPLORED ROOMS
	stanze = []
	spazi = []
	spazi_parziali = []
	indici_stanze_parziali = []
	for s in plan_o.spazi:
		if s.out == False: # IF THE ROOM IS CONNECTED TO SOMETHING
			stanze.append(s.spazio)
			spazi.append(s)
		if s.parziale == False:
			stanze_complete.append(s.spazio)
		else:
			spazi_parziali.append(s)
			
	for i in spazi_parziali:
		logger.append("PREDICTED AREA FOR FRONTIER "+str(i.cluster_frontiera) + " is: " + str(i.area_predetta))
		logger.append("FRONTIER IS CLOSE TO CONTOUR: "+ str(i.tocco_bordo))
		logger.append("CELLS ARE: " + str(len(i.cells)))
		for celletta in i.cells:
			logger.append("UNKNOWN PIXEL OF CELLS ARE: "+ str(celletta.grey_pixel_count))
		logger.append("--------------------")
	end_timer = time.time()
	logger.append("TOTAL TIME: " +str(end_timer-start_timer))
	if (par.SAVELOGGER or frontierObject.saveMap) and not par.DISEGNA:
		try:
			dsg.disegna_sovrapposizione_predizione(copia_spazi, plan_o.spazi, contours , dsg.get_colors(plan_o.spazi), xmin, ymin, xmax, ymax, filepath = path_obj.filepath, format='png')
		except Exception as e:
			logger.append("error in drawing sovrapposizione_predizione: " + str(e))

	if par.DISEGNA:
		id_stanze = xmlutils.crea_xml(path_obj.filepath+'18_xml_segmentazione.xml',spazi_parziali)#id stanze e' ordinato come spazi e possiede l'id univo assegnato alla stanza
		try:
			dsg.disegna_sovrapposizione_predizione(copia_spazi, plan_o.spazi, contours , dsg.get_colors(plan_o.spazi), xmin, ymin, xmax, ymax, filepath = path_obj.filepath, format='png')
		except:
			logger.append("error in drawing sovrapposizione_predizione")
		SAVE_LOGGERFILE = path_obj.filepath + '/logger.txt'
		with open(SAVE_LOGGERFILE,'w+') as LOGGERFILE:
			print >> LOGGERFILE, "Log: "
			for part in logger:
				print >> LOGGERFILE, part


	plt.clf()
	plt.cla()
	plt.close('all')
	return spazi_parziali,logger

#---------------------------------------------------------------------------
#-----CONVERT FROM PGM IMAGE TO PNG IMAGE, AND STRETCHES THE IMAGES---------
#---------------------------------------------------------------------------
def convert_imm(src_im, out_file,da_ruotare, angle = 45, prima_dimensione = 1000,):
	start_timer = time.time()
	#compute  size 
	real_size = src_im.size
	proportion = float(real_size[0])/real_size[1]
	seconda_dimensione = float(prima_dimensione)/proportion
	size = int(real_size[0]),int (real_size[1])
	dst_im = Image.new("RGBA", size, "red" )
	im = src_im.convert('RGBA')
	if da_ruotare:
		rot = im.rotate( angle, expand=1 ).resize(size)
	else:
		rot = im.resize(size)
	dst_im.paste( rot, (0, 0), rot )
	plt.clf()
	plt.cla()
	plt.close()
	#Test to convert the image without saving it.
	numpy_image=np.array(dst_im)  
	img_rgb=cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR) 
	#original
	
	altezza = len(img_rgb)
	larghezza = len(img_rgb[0])
	for i in xrange(0,altezza):
		for j in xrange(0,larghezza):
			if img_rgb[i][j][0]==0  and img_rgb[i][j][1] ==0 and img_rgb[i][j][2]==255:
				img_rgb[i][j][0]=205
				img_rgb[i][j][1]=205
				img_rgb[i][j][2]=205
	
	fig = plt.figure(frameon=False)
	ax = plt.Axes(fig, [0., 0., 1., 1.])
	ax.set_axis_off()
	fig.add_axes(ax)
	ax.imshow(img_rgb, aspect='auto')
	
	buf = io.BytesIO()
	fig.savefig(buf, format='png')
	buf.seek(0)
	temp_im = Image.open(buf)
	src_im = temp_im.copy()
	buf.close()
	
	im = src_im.convert('RGBA')
	rot = im
	dst_im = im

	if (par.DISEGNA):
		dst_im.save(out_file)
	if par.PRINTTIMER:
		print "MAP TO IMAGE FOR PREDICTION TOOK ", time.time()-start_timer, " SECONDS"
	return dst_im

def convert_imm2(src_im, out_file,da_ruotare, angle = 45, prima_dimensione = 1000,):
	start_timer = time.time()
	real_size = src_im.size
	size = int(real_size[0]),int (real_size[1])
	dst_im = Image.new("RGBA", size, "red" )
	im = src_im.convert('RGBA')
	if da_ruotare:
		rot = im.rotate( angle, expand=1 )#.resize(size)
	else:
		rot = im#.resize(size)
	dst_im.paste( rot, (0, 0), rot )
	plt.clf()
	plt.cla()
	plt.close()
	
	numpy_image=np.array(dst_im)  
	img_rgb=cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR) 
	altezza = len(img_rgb)
	larghezza = len(img_rgb[0])
	for i in xrange(0,altezza):
		for j in xrange(0,larghezza):
			if img_rgb[i][j][0]==0  and img_rgb[i][j][1] ==0 and img_rgb[i][j][2]==255:
				img_rgb[i][j][0]=205
				img_rgb[i][j][1]=205
				img_rgb[i][j][2]=205
	
	img = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
	src_im = Image.fromarray(img)
	im = src_im.convert('RGBA')
	rot = im#.resize(size)
	dst_im = im

	if (par.DISEGNA):
		dst_im.save(out_file)
	if par.PRINTTIMER:
		print "L'adattamento della mappa ha impiegato ", time.time()-start_timer, " secondi"
	return dst_im

def convert_imm3(src_im, out_file,da_ruotare, angle = 45, prima_dimensione = 1000,):
	start_timer = time.time()
	dst_im = src_im.convert('RGBA')

	if (par.DISEGNA):
		dst_im.save(out_file)
	if par.PRINTTIMER:
		print "L'adattamento della mappa ha impiegato ", time.time()-start_timer, " secondi"
	return dst_im

def main():
	start_time = time.time()
	print "Analysis Started"
	parameter_obj, path_obj = read_parameter()
	frontierObject = fo.FrontierObject(False,[],[],[],[])
	if par.AZIONE == 'batch':
		print "Batch"
		if (par.MAPPADACONVERTIRE):
			pattern_to_search = path_obj.INFOLDERS+'*.pgm'
		else:
			pattern_to_search = path_obj.INFOLDERS+'*.png'
		for metricMap in glob.glob(pattern_to_search):
			print "Parso: ", metricMap
			path_obj.metricMap = metricMap
			map_name = metricMap.split('/')[-1][:-4]
			SAVE_FOLDER = path_obj.OUTFOLDERS+'/'+map_name
			if not os.path.exists(SAVE_FOLDER):
				os.mkdir(SAVE_FOLDER)
				path_obj.filepath = SAVE_FOLDER+'/'
				if par.MAPPADACONVERTIRE:
					img_to_be_converted = Image.open(path_obj.metricMap)
					converted_img = convert_imm(img_to_be_converted, path_obj.filepath+par.CONVERTEDNAME,par.DARUOTARE)
					# use numpy to convert the pil_image into a numpy array
					numpy_image=np.array(converted_img)
					# convert to a openCV2 image, notice the COLOR_RGB2BGR which means that
					# the color is converted from RGB to BGR format
					opencv_image=cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR) 
					if par.PRINT:
						print "Conversione completata"
				else:
					opencv_image = cv2.imread(path_obj.metricMap)
				our_time = str(dt.datetime.now())[:-10].replace(' ','@') #get current time
				SAVE_FOLDER = os.path.join('./log', our_time)
				if not os.path.exists(SAVE_FOLDER):
					os.mkdir(SAVE_FOLDER)
				SAVE_LOGFILE = SAVE_FOLDER+'/log.txt'
			#---------------------------------------------------------
				with open(SAVE_LOGFILE,'w+') as LOGFILE:
					if par.PRINT:
						print "Created logfile"
					print >>LOGFILE, "Start at " ,dt.datetime.now()
					start_analysis(parameter_obj,path_obj,opencv_image,frontierObject)
					LOGFILE.flush()				
					print >> LOGFILE, "End at ", dt.datetime.now()
			else:
				print "Skip"
			
	end_time = time.time()
	elapsed = end_time - start_time
	print "Ended after ", elapsed," seconds."


#------------------------------------------------------
#---similar to main: Used when called from ROS Node:---
#------------------------------------------------------
def floorplan_analyzer (img_received,frontierObject,counter):
	start = time.time()
	parameter_obj, path_obj = read_parameter()
	#--convert from cv2 to PIL image
	#cv_image = img_received.copy()
	#img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
	#pil_image = Image.fromarray(img)
	pil_image = img_received.copy()	
	#----------------------------
	#--Adapt the received photo to our script
	converted_img = convert_imm3(pil_image, par.CONVERTEDNAME,False)
	#converted_img = pil_image.convert('RGBA')
	# use numpy to convert the pil_image into a numpy array
	numpy_image=np.array(converted_img)
	# convert to a openCV2 image, notice the COLOR_RGB2BGR which means that 
    # the color is converted from RGB to BGR format
	opencv_image=cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR)
	#opencv_image = img_received
	#par.DISEGNA = False
	#par.PRINT = False
	par.PRINTTIMER = False
	#---------------------------------------
	if par.PRINT:
		print "Start analysis"
	if par.DISEGNA or par.SAVEMAP or par.SAVELOGGER or frontierObject.saveMap:
		SAVE_FOLDER = os.environ['HOME']+"/Desktop/SavedMap/Map"+str(counter)
		path_obj.filepath = SAVE_FOLDER+'/'
	spazi_parziali,logger = start_analysis(parameter_obj,path_obj,opencv_image,frontierObject)
	res = frontierObject.getGrid().info.resolution
	aree_list = []
	frontier_list = []
	touch_edges = []
	aree_grey_pixel_list = []
	for i in spazi_parziali:
		aree_list.append(i.area_predetta*res)
		frontier_list.append(i.cluster_frontiera)
		touch_edges.append(i.tocco_bordo)
		area_grey_pixel = 0
		for c in i.cells:
			area_grey_pixel+=c.grey_pixel_count
		aree_grey_pixel_list.append(area_grey_pixel*res)
		logger.append("L'area in grey pixel della frontiera " +str(i.cluster_frontiera) + " e': " +str(area_grey_pixel))
	end = time.time()
	return logger,aree_list,frontier_list,touch_edges,aree_grey_pixel_list
#------------------------------------------------------
#-----Read Parameters as object from script python-----
#------------------------------------------------------

def read_parameter ():
	parameter_obj = par.Parameter_obj()
	path_obj = par.Path_obj()
	return parameter_obj, path_obj

#--------------------------------
#--------------------------------
#--------------------------------
if __name__ == '__main__':
	main()
