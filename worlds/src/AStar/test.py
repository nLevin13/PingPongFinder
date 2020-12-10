import numpy as np
from PIL import Image
from sys import argv as args
from cv2 import imread, resize, imwrite

def get_obstacle_array(img_name):
	img = Image.open(img_name)
	array = np.flip(np.array(img), axis=0)
	array = resize(array, None, fx=0.25, fy=0.25)
	if len(array.shape) > 2:
		array = array[:,:,0]
	ox, oy = [], []
	for i in range(array.shape[0]):
		for j in range(array.shape[1]):
			if array[i][j] < 255:
				ox.append(i * 4)
				oy.append(j * 4)
	return oy, ox

if __name__ == "__main__":
	img = imread('/home/kyletucker/ros_workspaces/project/src/stdr_simulator/stdr_resources/maps/sparse_obstacles.png')
	#array = np.array(img)
	print(img.shape)
	img = resize(img, None, fx=0.25, fy=0.25)
	print(img.shape)

	# img[160][20] = 0
	# img[15][100] = 0

	img2 = Image.fromarray(img)
	img2.save('testrgb3.png')
