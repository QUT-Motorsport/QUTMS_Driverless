#importing libraries

#!/usr/bin/env python3.8
import torch
import cv2

# main yolov5 detector
def yolov5_detector(model, image):

	# Classes: Blue 0, Yellow 1, Orange 2
	#inference
	model.conf = 0.25
	results = model(image)


	image_an = cv2.imread(image)

	# results.show()
	# cv2.waitKey(0)

	return results

#main function for individual use
def main():
	#loading the model
	model = torch.hub.load('ultralytics/yolov5', 'custom', path='C:/Users/BRAYTH/Documents/QUT/MotorSport/yoloAnt/yoloAnt/Scripts/YBV1.pt')
	#test image
	image1 = 'C:/Users/BRAYTH/Documents/QUT/MotorSport/yoloAnt/yoloAnt/Scripts/av5/av5 (217).jpg'

	# Running Inference
	data = yolov5_detector(model, image1)

	# Reading and Getting Dimensions
	image_gt = cv2.imread(image1)
	dimensions = image_gt.shape
	height = dimensions[0]
	width = dimensions[1]

	# Converting to HSV
	image_gt_HSV = cv2.cvtColor(image_gt, cv2.COLOR_BGR2HSV)

	# ORANGE_HSV_THRESH = Threshold(lower=[0, 100, 50], upper=[15, 255, 255])
	print(len(data.xyxy[0]))

	for i in range(len(data.xyxy[0])):
		yolo_cone_colour = int(data.xyxy[0][i][5])

		# coords (0, 1) Top left (x, y)
		# 		 (2, 3) Bottom Right (x, y)

		# Getting the centre of a cone
		cone_center_x = int(data.xyxy[0][i][0]) + int((int(data.xyxy[0][i][2]) - int(data.xyxy[0][i][0])) / 2)
		cone_center_y = int(data.xyxy[0][i][1]) - int((int(data.xyxy[0][i][1]) - int(data.xyxy[0][i][3])) / 2)

		# Excludes Cones that are on the border of the image
		if (width - cone_center_x > 20) and (0 + cone_center_y > 20) and (height - cone_center_y > 20) and (0 + cone_center_y > 20):

			# Getting HSV value of the cone and true colour
			ground_truth_colour = check_HSV(image_gt_HSV, cone_center_x, cone_center_y)

			# Checking if yolo thinks its blue
			if ground_truth_colour == 'blue' and yolo_cone_colour != 0:
				yolo_cone_colour = 0

			if ground_truth_colour == 'yellow' and yolo_cone_colour != 1:
				yolo_cone_colour = 1

			# Resulting bounding boxes
			if yolo_cone_colour == 0:
				cv2.rectangle(image_gt, (cone_center_x, cone_center_y),	(cone_center_x, cone_center_y), (255, 0, 0), 3)

			if yolo_cone_colour == 1:
				cv2.rectangle(image_gt, (cone_center_x, cone_center_y), (cone_center_x, cone_center_y), (0, 255, 255), 3)

	cv2.imshow('Cone Locations', image_gt)
	cv2.waitKey(0)

def check_HSV(image_HSV, cone_center_x, cone_center_y):

	true_colour = ''
	H, S, V = image_HSV[cone_center_y, cone_center_x]
	print('CV2 HSV: '+ str(H), str (S), str(V))
	H_useful = round(H / 180 * 360, 2)
	S_useful = round(S / 255, 2)
	V_useful = round(V / 255, 2)
	print('H: ' + str(H_useful) + ' S: ' + str(S_useful) + '% V: ' + str(V_useful) + '%')

	# HSV threshold constants
	YELLOW_HSV_THRESH_lower = [25, 80, 90]
	YELLOW_HSV_THRESH_upper = [35, 255, 255]
	BLUE_HSV_THRESH_lower = [95, 26, 12]
	BLUE_HSV_THRESH_upper = [135, 255, 255]

	if YELLOW_HSV_THRESH_lower[0] <= H <= YELLOW_HSV_THRESH_upper[0] and YELLOW_HSV_THRESH_lower[1] <= S <= \
			YELLOW_HSV_THRESH_upper[1] and YELLOW_HSV_THRESH_lower[2] <= V <= YELLOW_HSV_THRESH_upper[2]:
		true_colour = 'yellow'

	if BLUE_HSV_THRESH_lower[0] <= H <= BLUE_HSV_THRESH_upper[0] and BLUE_HSV_THRESH_lower[1] <= S <= \
			BLUE_HSV_THRESH_upper[1] and BLUE_HSV_THRESH_lower[2] <= V <= BLUE_HSV_THRESH_upper[2]:
		true_colour = 'blue'

	return true_colour


if __name__ == '__main__':
    main()

#######################################################################################################################
