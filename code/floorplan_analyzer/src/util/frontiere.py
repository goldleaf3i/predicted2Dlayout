import cv2
from matplotlib import pyplot as plt
from skimage.feature import blob_dog
import math
from sklearn.cluster import DBSCAN
import numpy as np
import matplotlib.cm as cmx
import matplotlib
import random
import collections

class pixel(object):
	def __init__(self,r,g,b,x,y):
		self.rgb = [r,g,b]
		self.x = x
		self.y =y 
	def set_cluster(self,c):
		self.cluster = c

def cerca_frontiere_immagine(img_rgb):
	'''
	ricerco le frontiere e le coloro, prendo tutti i pixel bianchi che confinano con un pixel settato a 205(che pare essere il colore primario)
	'''
	new_imm = img_rgb.copy()
	altezza = len(new_imm)
	larghezza = len(new_imm[0])
	
	#prendo il primo pixel dell'immagine, in teoria e' sempre una parte sconosciuta (dovrebbe essere 205)
	r1 = img_rgb[0][0][0]
	g1 = img_rgb[0][0][1]
	b1 = img_rgb[0][0][2] 
	
	lista_pixel_frontiera = []
	for i in xrange( 0, altezza):
		for j in xrange( 0, larghezza):
			r = new_imm[i][j][0]
			g = new_imm[i][j][1]
			b = new_imm[i][j][2]
			if	r>206 and g>206 and b>206:
				#controllo che confini con un pixel grigio
				p1 = [new_imm[i-1][j-1][0], new_imm[i-1][j-1][1], new_imm[i-1][j-1][2]]
				p2 = [new_imm[i-1][j][0], new_imm[i-1][j][1], new_imm[i-1][j][2]]
				p3 = [new_imm[i-1][j+1][0], new_imm[i-1][j+1][1], new_imm[i-1][j+1][2]]
				p4 = [new_imm[i][j-1][0], new_imm[i][j-1][1], new_imm[i][j-1][2]]
				p5 = [new_imm[i][j+1][0], new_imm[i][j+1][1], new_imm[i][j+1][2]]
				p6 = [new_imm[i+1][j-1][0], new_imm[i+1][j-1][1], new_imm[i+1][j-1][2]]
				p7 = [new_imm[i+1][j][0], new_imm[i+1][j][1], new_imm[i+1][j][2]]
				p8 = [new_imm[i+1][j+1][0], new_imm[i+1][j+1][1], new_imm[i+1][j+1][2]]
				
				#prendo tutti quei pixel che confinano con il grigio
				if (p1[0]==r1 or p2[0]==r1 or p3[0]==r1 or p4[0]==r1 or p5[0]==r1 or p6[0]==r1 or p7[0]==r1 or p8[0]==r1) :
					#bordo trovato
					new_imm[i][j][0] = 0
					new_imm[i][j][1] = 255
					new_imm[i][j][2] = 0
					#pix = pixel(0,255,0,i,j)
					pix = pixel(0,255,0,i,j)
					lista_pixel_frontiera.append(pix)
			#controllo che nel dintorno di una bolla non ci siano parti occupate
	
	return new_imm, lista_pixel_frontiera

def crea_distance_matrix_frontiere(lista_pixel_frontiera):
	
	matrice = []
	for c1 in lista_pixel_frontiera:	
		row=[]
		for c2 in lista_pixel_frontiera:
			#sono da confrontare
			d1 = math.pow((c1.x-c2.x),2)
			d2 = math.pow((c1.y-c2.y),2)
			dist = math.sqrt((d1+d2))
			row.append(dist)
		
		matrice.append(row)
		
	return matrice

def DBscan_per_pixel(dist_mat, eps = 5, min_samples = 10):# eps = 20
	'''
	creo la lista di frontiere
	'''
	
	db = DBSCAN(eps, min_samples, metric='precomputed').fit(dist_mat)
	labels = db.labels_

	# Number of clusters in labels, ignoring noise if present.
	n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
	print('Estimated number of clusters: %d' % n_clusters_)
	
	return labels, n_clusters_


def get_colori_rgb(labels):
	'''
	restituisce un insieme di colori differenti per ogni cluster 
	'''
	colori_assegnati = []	
	for label in set(labels):
		color= [int(random.random()*255), int(random.random()*255), int(random.random()*255)]
		colori_assegnati.append(color)
	return colori_assegnati


def disegna_frontiere(frontiere, lista_pixel_frontiera, labels, colori):
	'''
	disegna la mappa con le frontiere colorate
	cambio il colore ogni volta che cambio il cluter
	'''
	
	immagine_cluster= frontiere.copy()
	
	altezza = len(frontiere)
	larghezza = len(frontiere[0])
	
	#cluster_attuale= labels[0]
	
	#colori = get_colori_rgb(labels)
		
	
	
	for i in xrange( 0, altezza):
		for j in xrange( 0, larghezza):
			r = frontiere[i][j][0]
			g = frontiere[i][j][1]
			b = frontiere[i][j][2]
			if	r==0 and g==255 and b==0:
				for index, p in enumerate(lista_pixel_frontiera):
					if p.x == i and p.y == j:
						immagine_cluster[i][j][0] = colori[labels[index]][0]
						immagine_cluster[i][j][1] = colori[labels[index]][1]	
						immagine_cluster[i][j][2] = colori[labels[index]][2]
						p.rgb[0] = colori[labels[index]][0]
						p.rgb[1] = colori[labels[index]][1]
						p.rgb[2] = colori[labels[index]][2]			
	return immagine_cluster
	
def set_cluster_pixel(labels, lista_pixel_frontiera):
	'''
	setto il cluster appena trovato alla lista di pixel delle frontiere
	'''
	for l,p in zip(labels, lista_pixel_frontiera):
		p.set_cluster(l)
	

def elimina_cluster_frontiere_inutili(labels,frontiere, lista_pixel_frontiera, cluster_da_eliminare):
	'''
	eliminino il cluster delle frontiere inutili, come ad esempio il cluster degli outliers e tutti i cluster piccoli
	'''
	#elimino gli outliers(non sono interessanti)
	#clono la lista delle labels di dbscan per eliminare il cluster degli outliers
	labels_cloned = labels[:]
	frontiere_cloned = frontiere.copy()
	lista_pixel_frontiere_cloned = lista_pixel_frontiera[:]
	
	#1)nell'immagine con le frontiere colorate di verde setto come neri quei pixel classificati come outliers.
	altezza = len(frontiere_cloned)
	larghezza = len(frontiere_cloned[0])
	for i in xrange( 0, altezza):
		for j in xrange( 0, larghezza):
			r = frontiere_cloned[i][j][0]
			g = frontiere_cloned[i][j][1]
			b = frontiere_cloned[i][j][2]
			if	r==0 and g==255 and b==0:
				for index, p in enumerate(lista_pixel_frontiera):
					if p.x == i and p.y == j and (p.cluster in cluster_da_eliminare):
						frontiere_cloned[i][j][0]= 0
						frontiere_cloned[i][j][1]= 0
						frontiere_cloned[i][j][2]= 0
		
	#2)elimino da label il cluster da eliminare
	labels_cloned = [x for x in labels_cloned if (x not in cluster_da_eliminare)]
	#3)elimino da lista_pixel_frontiere_cloned il cluster da eliminare
	lista_pixel_frontiere_cloned = [x for x in lista_pixel_frontiere_cloned if (x.cluster not in cluster_da_eliminare)]
	
	return frontiere_cloned, labels_cloned, lista_pixel_frontiere_cloned



def ottieni_frontire_principali(immagine, min_n_pixel_stesso_cluster= 100):
	'''
	ottengo le frontiere prinipali della mappa. clusterizzo ogni punto perche' faccia parte di un cluster specifico. 
	una volta ottenuti i clusters, seleziono i cluster piu' grandi ed elimino quelli piu' piccoli che ragionevolmente saranno solo rumore.
	
	restituisco la lista di labels, l'immagine grb con i pixel colorati in presenza dei cluster selezionati, la lista_pixel_frontiere_cloned (lista di oggetti Pixel) dove ci sono tutti i pixel dei cluster
	se non ci sono labels significa che non ci sono frontiere, dunque restituisco l'immagine originale e le liste vuote.
	'''
	
	img_rgb = immagine.copy()

	frontiere, lista_pixel_frontiera = cerca_frontiere_immagine(img_rgb)
	dist_mat = crea_distance_matrix_frontiere(lista_pixel_frontiera)
	labels, n_clusters_ = DBscan_per_pixel(dist_mat)#ogni pixel della frontiera viene clusterizzato con DBscan
	colori = get_colori_rgb(labels)#ottengo i colori necessari
	if len(labels)>0:
		immagine_cluster = disegna_frontiere(frontiere,lista_pixel_frontiera, labels, colori) #disegno nuovamente la mappa cambiando i colori in base al cluster.
	
		set_cluster_pixel(labels, lista_pixel_frontiera)#setto il cluster ai pixel 

		#-----elimino clusters--------------------------------------------------------------------
		#elimino i cluster inutili. In particolare il cluster -1
		frontiere_cloned, labels_cloned, lista_pixel_frontiere_cloned = elimina_cluster_frontiere_inutili(labels,frontiere, lista_pixel_frontiera, [-1])
		#ridisegno l'immagine senza gli outliers
		if len(labels_cloned)>0:
			immagine_cluster = disegna_frontiere(frontiere_cloned,lista_pixel_frontiere_cloned, labels_cloned, colori) #disegno nuovamente la mappa cambiando i colori in base al cluster.

			#elimino tutti quei cluster che hanno un numero inferiore a 100 pixel nello stesso cluster
			#conto le occorrenze 
			counter=collections.Counter(labels_cloned)
			print(counter)
			cluster_piccoli= []
			for k in counter.keys():
				if counter[k] < min_n_pixel_stesso_cluster: #TODO: fai diventare il 100 un parametro 
					cluster_piccoli.append(k)
			print cluster_piccoli

			#elimino i cluster piccoli
			frontiere_cloned, labels_cloned, lista_pixel_frontiere_cloned = elimina_cluster_frontiere_inutili(labels_cloned,frontiere_cloned, lista_pixel_frontiere_cloned, cluster_piccoli)
			if len(labels_cloned)>0:
				immagine_cluster = disegna_frontiere(frontiere_cloned,lista_pixel_frontiere_cloned, labels_cloned, colori) #disegno nuovamente la mappa cambiando i colori in base al cluster.
			else:
				immagine_cluster = img_rgb
				frontiere_cloned = img_rgb
				labels_cloned=[]
				lista_pixel_frontiere_cloned = []
			#-----------------------------------------------------------------------------------------
		else:
			immagine_cluster = img_rgb
			frontiere_cloned = img_rgb
			labels_cloned=[]
			lista_pixel_frontiere_cloned = []
	else:
		immagine_cluster = img_rgb
		frontiere_cloned = img_rgb
		labels_cloned=[]
		lista_pixel_frontiere_cloned = []
	return immagine_cluster, frontiere_cloned, labels_cloned, lista_pixel_frontiere_cloned

