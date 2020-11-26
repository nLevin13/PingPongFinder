import numpy as np
from PIL import Image
from sys import argv as args
from cv2 import imread, resize, imwrite

def get_obstacle_array(img_name):
	img = Image.open(img_name)
	array = np.array(img)
	ox, oy = [], []
	for i in range(array.shape[0]):
		for j in range(array.shape[1]):
			if array[i][j] < 255:
				ox.append(i)
				oy.append(j)
	return ox, oy

if __name__ == "__main__":
	img = imread(args[1])[...,0]
	#array = np.array(img)
	print(img.shape)
	img = resize(img, None, fx=0.25, fy=0.25)
	print(img.shape)

	# img[160][20] = 0
	# img[15][100] = 0

	img2 = Image.fromarray(img)
	img2.save('testrgb3.png')