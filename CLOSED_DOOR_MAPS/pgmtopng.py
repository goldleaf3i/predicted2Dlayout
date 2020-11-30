import PIL
from PIL import Image
from os import listdir
from os.path import join, isdir
import statistics as st
from PIL import ImageFont
from PIL import ImageDraw


pathname = "./"
image1name = "1-Initial_pgm.pgm"
image2name = "Final.pgm"
logfilename = "iou_migliore_con_bordo.log"
image3name = "pgm.png"
for fold in listdir(pathname):
	#check if is a directory
	is_dir = isdir(fold)

	if is_dir:
		print(fold)
		subfold = join(pathname,fold)
		#print(subfold,listdir(subfold))
		image3file = join(subfold,image3name)
		save3file="./"+fold+"_GT.png"
		Image.open(image3file).save(save3file)
		for fold_i in listdir(subfold) :
			subpath = join(subfold,fold_i)
			if isdir(subpath) and "doors" in subpath :
				
				image1file = join(subpath,fold,image1name)
				image2file = join(subpath,fold,image2name)
				save1file = "./"+fold+"_"+fold_i+"_IN.png"
				save2file = "./"+fold+"_"+fold_i+"_OUT.png"
				print(subpath,image2file,image1file,save1file,save2file)

				Image.open(image1file).save(save1file)
				Image.open(image2file).save(save2file)
				#logfilepath = join(subpath,fold,logfilename)
				#logfile = open(logfilepath,"r")
				#print(logfile.read())