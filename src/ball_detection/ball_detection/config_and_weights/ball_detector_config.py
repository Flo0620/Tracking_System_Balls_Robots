#!/usr/bin/env python


ball_radius = 0.0725 #The radius of the balls that should be tracked in meter.
max_num_balls = -1#The maximum number of balls that can be in the image. Only the best \"max_num_balls\" detections will be used. -1 means that the number is unknown and all detections above the conf_thresh will be used.
conf_thresh = 0.75 #Detections with a confidence less than the conf_thresh will be discarded.
nms_thresh = 0.5 #The threshold used in the non-maximal suppresion
