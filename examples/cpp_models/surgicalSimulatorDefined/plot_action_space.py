import numpy as np
import matplotlib.pyplot as plt
import cv2


astar_actions = [((30, 100, -10), 4), ((30, 115, -10), 3), ((30, 130, -10), 1), ((30, 145, -10), 1), ((30, 160, -10), 1), ((45, 100, -10), 1), ((45, 115, -10), 3), ((45, 130, -10), 3), ((45, 145, -10), 1), ((45, 160, -10), 1), ((60, 100, -10), 4), ((60, 115, -10), 3), ((60, 130, -10), 3), ((60, 145, -10), 3), ((60, 160, -10), 3), ((75, 100, -10), 0), ((75, 115, -10), 3), ((75, 130, -10), 3), ((75, 145, -10), 3), ((75, 160, -10), 3), ((90, 100, -10), 0), ((90, 115, -10), 3), ((90, 130, -10), 3), ((90, 145, -10), 3), ((90, 160, -10), 3), ((30, 100, -5), 5), ((30, 115, -5), 3), ((30, 130, -5), 1), ((30, 145, -5), 1), ((30, 160, -5), 1), ((45, 100, -5), 1), ((45, 115, -5), 3), ((45, 130, -5), 3), ((45, 145, -5), 1), ((45, 160, -5), 1), ((60, 100, -5), 0), ((60, 115, -5), 3), ((60, 130, -5), 3), ((60, 145, -5), 3), ((60, 160, -5), 3), ((75, 100, -5), 0), ((75, 115, -5), 3), ((75, 130, -5), 3), ((75, 145, -5), 3), ((75, 160, -5), 3), ((90, 100, -5), 0), ((90, 115, -5), 3), ((90, 130, -5), 3), ((90, 145, -5), 3), ((90, 160, -5), 3), ((30, 100, 0), 5), ((30, 115, 0), 3), ((30, 130, 0), 1), ((30, 145, 0), 1), ((30, 160, 0), 1), ((45, 100, 0), 1), ((45, 115, 0), 3), ((45, 130, 0), 3), ((45, 145, 0), 1), ((45, 160, 0), 1), ((60, 100, 0), 0), ((60, 115, 0), 3), ((60, 130, 0), 3), ((60, 145, 0), 3), ((60, 160, 0), 3), ((75, 100, 0), 0), ((75, 115, 0), 3), ((75, 130, 0), 3), ((75, 145, 0), 3), ((75, 160, 0), 3), ((90, 100, 0), 0), ((90, 115, 0), 3), ((90, 130, 0), 3), ((90, 145, 0), 3), ((90, 160, 0), 3), ((30, 100, 5), 5), ((30, 115, 5), 3), ((30, 130, 5), 1), ((30, 145, 5), 1), ((30, 160, 5), 1), ((45, 100, 5), 1), ((45, 115, 5), 3), ((45, 130, 5), 3), ((45, 145, 5), 1), ((45, 160, 5), 1), ((60, 100, 5), 0), ((60, 115, 5), 3), ((60, 130, 5), 3), ((60, 145, 5), 3), ((60, 160, 5), 3), ((75, 100, 5), 0), ((75, 115, 5), 3), ((75, 130, 5), 3), ((75, 145, 5), 3), ((75, 160, 5), 3), ((90, 100, 5), 0), ((90, 115, 5), 3), ((90, 130, 5), 3), ((90, 145, 5), 3), ((90, 160, 5), 3), ((30, 100, 10), 5), ((30, 115, 10), 3), ((30, 130, 10), 1), ((30, 145, 10), 1), ((30, 160, 10), 1), ((45, 100, 10), 1), ((45, 115, 10), 3), ((45, 130, 10), 4), ((45, 145, 10), 4), ((45, 160, 10), 4), ((60, 100, 10), 4), ((60, 115, 10), 4), ((60, 130, 10), 4), ((60, 145, 10), 4), ((60, 160, 10), 4), ((75, 100, 10), 4), ((75, 115, 10), 4), ((75, 130, 10), 4), ((75, 145, 10), 4), ((75, 160, 10), 4), ((90, 100, 10), 4), ((90, 115, 10), 4), ((90, 130, 10), 4), ((90, 145, 10), 4), ((90, 160, 10), 4), ]


num_images = 5
img_height = 200
img_width = 200
points_per_img = 25
total_num_actions = 6

# colors corresponding to actions - x:red, y:blue, theta:green
images = np.ones((num_images, img_height, img_width, 3))*255 # array of 5: 200 by 200 images


def action_num_to_arrow(action_num):
	if (action_num == 0):
		# right arrow 
		return ">"
	elif (action_num == 1):
		# left arrow 
		return "<"
	elif  (action_num == 2):
		# up arrow
		return "^"
	elif (action_num == 3):
		# down arrow 
		return "V"
	elif (action_num == 4):
		# theta up 
		return "T"
	elif (action_num == 5):
		# theta down 
		return "t"
	else:
		print("THERE WAS AN ERROR")

# index maps to action
colors = [(255, 0, 0), (255, 255, 0), (0, 255, 0), (0, 255, 255), (200, 200, 255), (255, 0, 255)]
use_arrows = True

for img_num in range(num_images):
	for idx in range(points_per_img):
		list_idx = idx + 25*img_num
		action = astar_actions[list_idx][1]

		# only if it is a valid action
		if (action >= 0 and action < total_num_actions):
			coords = (astar_actions[list_idx][0][0], astar_actions[list_idx][0][1])
			color = colors[action]
			radius = 5
			thickness = -1

			cv2.circle(images[img_num], coords, radius, color,  thickness)

#flip all images
for i in range(num_images):
	images[i] = images[i][::-1, ::, ::]

# put the numbers right side up in the correct images. 
for img_num in range(num_images):
	for idx in range(points_per_img):
		list_idx = idx + 25*img_num
		# font 
		font = cv2.FONT_HERSHEY_SIMPLEX 
		fontScale = 0.3
		fontcolor = (0, 0, 0)
		fontThickness = 1
		coords = (astar_actions[list_idx][0][0], astar_actions[list_idx][0][1])
		fontCoords = (coords[0] - 4, img_height - coords[1] + 4)
		# Using cv2.putText() method 
		if not use_arrows: 
			cv2.putText(images[img_num], str(astar_actions[list_idx][1]) , fontCoords, font,  
		                   fontScale, fontcolor, fontThickness, cv2.LINE_AA) 
		else: 
			arrow = action_num_to_arrow(astar_actions[list_idx][1])
			cv2.putText(images[img_num], arrow, fontCoords, font, fontScale, fontcolor, fontThickness, cv2.	LINE_AA)


label_img = np.ones((200, 200, 3))*255
for i in range(len(colors)):
	coords = ((i+1)*20, 100)
	color = colors[i]
	radius = 5
	thickness = -1

	cv2.circle(label_img, coords, radius, color,  thickness)
	# font 
	font = cv2.FONT_HERSHEY_SIMPLEX 
	fontScale = 0.3
	fontcolor = (0, 0, 0)
	fontThickness = 1
	fontCoords = (coords[0] - 4, coords[1] + 4)
	# Using cv2.putText() method 
	if not use_arrows: 
		cv2.putText(label_img, str(i) , fontCoords, font,  
				fontScale, fontcolor, fontThickness, cv2.LINE_AA) 
	else: 
		arrow = action_num_to_arrow(i)
		cv2.putText(label_img, arrow, fontCoords, font, fontScale, fontcolor, fontThickness, cv2.LINE_AA)
"""
# flip the label image
label_img = label_img[::-1, ::, ::]

for i in range(len(colors)):

	coords = ((i+1)*20, 100)
	# font 
	font = cv2.FONT_HERSHEY_SIMPLEX 
	fontScale = 0.2
	fontcolor = (0, 0, 0)
	fontThickness = 1
	fontCoords = (coords[0] - 4, coords[1] + 4)
	# Using cv2.putText() method 
	cv2.putText(label_img, str(i) , fontCoords, font,  
				fontScale, fontcolor, fontThickness, cv2.LINE_AA) 
"""
# put the numbers in the label image correctly

# create all the plots
w = 10
h = 10
fig = plt.figure(figsize = (1, 5))
cols = 3
rows = 2

ax = []
for i in range(1, cols*rows + 1):
	if (i > 5):
		ax.append(fig.add_subplot(rows, cols, i))
		ax[-1].set_title("label image")
		plt.imshow(label_img.astype(np.int))
	else: 
		img = images[i - 1]
		ax.append(fig.add_subplot(rows, cols, i))
		ax[-1].set_title("theta: " + str(astar_actions[(i-1)*25][0][2]))
		plt.imshow(img.astype(np.int))
plt.show()