#!/usr/bin/env python
"""Extract images from a rosbag.
"""

from __future__ import print_function, division, with_statement

import os
import errno
import argparse
import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tqdm import tqdm

def mkdir(p):
    try:
        os.makedirs(p)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

def main():
    argparser = argparse.ArgumentParser(description='Extract images from a rosbag.')
    argparser.add_argument('bag_path', help='path to .bag file')
    argparser.add_argument('--left_image_topic', help='left camera image topic name (type: sensor_msgs/Image)', default = '/stereo/left/image_raw')
    argparser.add_argument('--right_image_topic', help='right camera image topic name (type: sensor_msgs/Image)', default = '/stereo/right/image_raw')
    argparser.add_argument('--encoding', help='new encoding (mono8, mono16, bgr8, rgb8, bgra8, rgba8)', default = 'bgr8')
    argparser.add_argument('--outdir', '-o', help='output dirname', default='')
    args = argparser.parse_args()

    print('INFO: used topic names...')
    print('....: left_image_topic:{0:>30s}'.format(args.left_image_topic))
    print('....: right_image_topic:{0:>30s}'.format(args.right_image_topic))

    bag_path = os.path.abspath(args.bag_path)
    keeped_topics = [
        args.left_image_topic,
        args.right_image_topic
        ]
    if args.outdir != '':
        outdir = os.path.abspath(args.outdir)
    else:
        outdir = os.path.join(os.path.dirname(bag_path),
                              os.path.basename(bag_path).split('.bag')[0])
    mkdir(outdir)

    limg_dir = os.path.join(
        outdir,
        args.left_image_topic.replace('/', '_')
        )
    mkdir(limg_dir)
    mkdir(os.path.join(limg_dir, 'data'))

    rimg_dir = os.path.join(
        outdir,
        args.right_image_topic.replace('/', '_')
        )
    mkdir(rimg_dir)
    mkdir(os.path.join(rimg_dir, 'data'))

    print('INFO: extract data to: {0}'.format(outdir))
    bridge = CvBridge()
    with rosbag.Bag(bag_path, mode='r') as bag:
        with tqdm(total=bag.get_message_count(topic_filters=keeped_topics)) as pbar:
            with open(os.path.join(limg_dir, 'timestamps.txt'), mode='w') as ts_txt:
                with open(os.path.join(limg_dir, 'filenames.txt'), mode='w') as fn_txt:
                    index = 0
                    for topic, msg, t in bag.read_messages(topics=[args.left_image_topic]):
                        file_name = '{0:08d}.png'.format(index)
                        ts_txt.write('{:.9f}\n'.format(msg.header.stamp.to_sec()))
                        fn_txt.write('{0}\n'.format(file_name))

                        png_path = os.path.join(
                            limg_dir, 'data', file_name
                            )
                        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding=args.encoding)
                        cv2.imwrite(png_path, cv_img)

                        index += 1
                        pbar.update()
            with open(os.path.join(rimg_dir, 'timestamps.txt'), mode='w') as ts_txt:
                with open(os.path.join(rimg_dir, 'filenames.txt'), mode='w') as fn_txt:
                    index = 0
                    for topic, msg, t in bag.read_messages(topics=[args.right_image_topic]):
                        file_name = '{0:08d}.png'.format(index)
                        ts_txt.write('{:.9f}\n'.format(msg.header.stamp.to_sec()))
                        fn_txt.write('{0}\n'.format(file_name))

                        png_path = os.path.join(
                            rimg_dir, 'data', file_name
                            )
                        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding=args.encoding)
                        cv2.imwrite(png_path, cv_img)

                        index += 1
                        pbar.update()

if __name__ == '__main__':
    main()
