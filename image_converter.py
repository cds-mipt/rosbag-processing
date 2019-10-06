#!/usr/bin/env python
from __future__ import print_function

import os
import sys
from glob import glob
from collections import Counter

import cv2
import rosbag
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import argparse
import wx
import math

def add_layout(layout, length, grid, board):
  space_width = int((layout[0].get('width') - board['x']) / grid['x'])
  space_height = int((layout[0].get('height') - board['y']) / grid['y'])

  for i in range(length):
    layout.append({'x': layout[0].get('x') + board['x']
																					 + int(i % grid['x']) * space_width,
                   'y': layout[0].get('y') + board['y']
									 												 + int(math.floor(i / grid['x'])) * space_height,
                   'width': space_width - board['x'],
                   'height': space_height - board['y']})

def set_layout(topics):
  dummy_app = wx.App(False)
  work_area = wx.Display(0).GetClientArea() # return wx.Rect
  layout = [{'x': work_area.x, 'y': work_area.y,
	           'width': work_area.width, 'height': work_area.height}]
  board = {'x':70, 'y':50}

  if len(topics) == 1:
    half_width = int((layout[0].get('width') - board['x']) / 2)
    half_height = int((layout[0].get('height') - board['y']) / 2)
    fourth_width = int(half_width / 2)
    fourth_height = int(half_height / 2)

    layout.append({'x': layout[0].get('x') + board['x'] + fourth_width,
                   'y': layout[0].get('y') + board['y'] + fourth_height,
                   'width': half_width - board['x'],
                   'height': half_height - board['y']})

  if len(topics) == 2:
    half_width = int((layout[0].get('width') - board['x']) / 2)
    half_height = int((layout[0].get('height') - board['y']) / 2)
    fourth_width = int(half_width / 2)

    layout.append({'x': layout[0].get('x') + board['x'] + fourth_width,
                   'y': layout[0].get('y') + board['y'],
                   'width': half_width - board['x'],
                   'height': half_height - board['y']})
    layout.append({'x': layout[0].get('x') + board['x'] + fourth_width,
                   'y': layout[0].get('y') + half_height + board['y'],
                   'width': half_width - board['x'],
                   'height': half_height - board['y']})

  elif len(topics) <= 4:
    add_layout(layout, len(topics), {'x':2, 'y':2}, board)

  elif len(topics) <= 6:
    add_layout(layout, len(topics), {'x':3, 'y':2}, board)

  elif len(topics) <= 9:
    add_layout(layout, len(topics), {'x':3, 'y':3}, board)
  print('Topics: ', topics)   
  print('Layout: ', layout)     
  for idx, topic in enumerate(topics):
    #idx = idx + 1
    cv2.namedWindow(topic, cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO | cv2.WINDOW_GUI_NORMAL)
    cv2.moveWindow(topic, layout[idx].get('x'), layout[idx].get('y'))
    cv2.resizeWindow(topic, layout[idx].get('width'), layout[idx].get('height'))
    cv2.waitKey(200)

def main_color_images(bagname, topics=[], encoding='passthrough', show_topic_names=False):
	bridge = CvBridge()
	with rosbag.Bag(bagname) as bag:
		# see http://wiki.ros.org/rosbag/Cookbook
		bag_topics = bag.get_type_and_topic_info()[1].keys()

		if show_topic_names:
			for tpc in bag_topics:
				print(tpc)
			return

		if not topics is None:
			for topic in topics:
				if not topic in bag_topics:
					print('error: there is no topic with name {} in bag, try -s flag'.format(topic))
					return
			set_layout(topics)
		else:
			set_layout(bag_topics)

		indexes = Counter()
		for topic, msg, _ in bag.read_messages(topics):

			# setup dataset structure:
			# <bag_name/topic_name>
			# |--data
			# |  |--img_000000.png
			# |  |--img_000001.png
			# |  |--..
			# |
			# |--filenames.txt
			# |--timestamps.txt

			main_path = bagname[:-4]
			if not os.path.exists(main_path):
					os.makedirs(main_path)

			topic_path = os.path.join(main_path, topic.lstrip('/'))
			if not os.path.exists(topic_path):
					os.makedirs(topic_path)

			data_path = os.path.join(topic_path, 'data')
			if not os.path.exists(data_path):
					os.makedirs(data_path)

			fname = 'img_{:06}.png'.format(indexes[topic])
			file_path = os.path.join(data_path, fname)
			indexes[topic] += 1

			timestamps_path = os.path.join(topic_path, 'timestamps.txt')
			filenames_path = os.path.join(topic_path, 'filenames.txt')

			with open(timestamps_path, "a") as f:
					f.write(str(msg.header.stamp) + '\n')

			with open(filenames_path, "a") as f:
					f.write('data/{}'.format(fname) + '\n')

			img = bridge.imgmsg_to_cv2(msg, encoding)
			cv2.imwrite(file_path, img)

			cv2.imshow(topic, img)
			cv2.waitKey(1)
			print('save ' + file_path)

def main_geodata(bagname, topics=[], encoding='passthrough', show_topic_names=False):
	bridge = CvBridge()
	with rosbag.Bag(bagname) as bag:
		# see http://wiki.ros.org/rosbag/Cookbook
		bag_topics = bag.get_type_and_topic_info()[1].keys()

		if show_topic_names:
			for tpc in bag_topics:
				print(tpc)
			return

		if not topics is None:
			for topic in topics:
				if not topic in bag_topics:
					print('error: there is no topic with name {} in bag, try -s flag'.format(topic))
					return
			set_layout(topics)
		else:
			set_layout(bag_topics)

		indexes = Counter()
		for topic, msg, _ in bag.read_messages(topics):

			# setup dataset structure:
			# <bag_name/topic_name>
			# |--data
			# |  |--img_000000.png
			# |  |--img_000001.png
			# |  |--..
			# |
			# |--filenames.txt
			# |--timestamps.txt

			main_path = bagname[:-4]
			if not os.path.exists(main_path):
					os.makedirs(main_path)

			topic_path = os.path.join(main_path, topic.lstrip('/'))
			if not os.path.exists(topic_path):
					os.makedirs(topic_path)

			data_path = os.path.join(topic_path, 'data')
			if not os.path.exists(data_path):
					os.makedirs(data_path)

			fname = str(topic)+'.img_{:06}.txt'.format(indexes[topic])
			file_path = os.path.join(data_path, fname)
			indexes[topic] += 1

			timestamps_path = os.path.join(topic_path, 'timestamps.txt')
			filenames_path = os.path.join(topic_path, 'filenames.txt')
			topic_data_path = os.path.join(data_path, fname)

			with open(topic_data_path, "w") as f:
					f.write(str(msg) + '\n')

			with open(timestamps_path, "a") as f:
					f.write(str(msg.header.stamp) + '\n')

			with open(filenames_path, "a") as f:
					f.write('data/{}'.format(fname) + '\n')

			print('save ' + str(fname))

if __name__ == '__main__':
	argparser = argparse.ArgumentParser(description='Save images from ROS bag')
	argparser.add_argument('bagname', help='path to bag file')
	argparser.add_argument('-s',
												 '--show_topic_names',
												 action='store_true',
												 help='show all topics name')
	argparser.add_argument('-t', '--topics', nargs='+', help='topic names')
	argparser.add_argument(
			'-e', '--encoding', default='passthrough', help='result encoding')
	args = argparser.parse_args()
	main_geodata(**vars(args))
