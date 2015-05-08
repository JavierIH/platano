# -*- coding: utf-8 -*-
"""
Created on Sat May  9 01:24:27 2015

@author: javierih
"""

import cv2
from Planner import Planner
from Simulation import Simulator

address='127.0.0.1'
port=19999
simulator = Simulator(address,port)

image = simulator.getImage()

th, image_bin = cv2.threshold(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 210, 255, cv2.THRESH_BINARY);

cv2.imshow("ok", image_bin)
cv2.waitKey(0)

#image = simulation.