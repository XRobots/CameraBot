#!/usr/bin/python3

import serial

import RPi.GPIO as GPIO
import jetson.inference
import jetson.utils

import time

import argparse
import sys

#setup serial
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)

# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.detectNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# load the object detection network
net = jetson.inference.detectNet(opt.network, sys.argv, opt.threshold)

# create video sources & outputs
input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv+is_headless)

# declare variables as global and that

global index
global width
global location
index = 0
width = 0
location = 0
flag = 0

# process frames until the user exits
while True:	

	# capture the next image
	img = input.Capture()

	# detect objects in the image (with overlay)
	detections = net.Detect(img, overlay=opt.overlay)

	# print the detections
	#print("detected {:d} objects in image".format(len(detections)))
	numberDet = (len(detections))
	#print(numberDet)

	if numberDet > 0:

                for detection in detections:
                        index = detections[0].ClassID
                        width = int((detections[0].Width))
                        location1 = int((detections[0].Center[0]))
                        location2 = int((detections[0].Top))

		# print index of item, width and horizonal location
                if index == 1:
                    print("detection:")
                    print(index)
                    print(width)
                    print(location1)
                    print(location2) 
                    chara = str(-20)
                    char1 = str(width)
                    char2 = str(location1)
                    char3 = str(location2) 
                    arduino.write(bytes(chara, 'utf8'))
                    arduino.write(bytes(',', 'utf8'))
                    arduino.write(bytes(char1, 'utf8'))
                    arduino.write(bytes(',', 'utf8'))
                    arduino.write(bytes(char2, 'utf8'))
                    arduino.write(bytes(',', 'utf8'))
                    arduino.write(bytes(char3, 'utf8'))
                    arduino.write(bytes(',', 'utf8'))
                else:
                    print("no detection")
                    chara = str(-40)
                    arduino.write(bytes(chara, 'utf8'))
                    arduino.write(bytes(',', 'utf8'))

	# render the image
	output.Render(img)

	# update the title bar
	output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

	# print out performance info
	#net.PrintProfilerTimes()

	# exit on input/output EOS
	if not input.IsStreaming() or not output.IsStreaming():
		break


