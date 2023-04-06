#!/usr/bin/env python

import argparse
import os.path as osp
import sys

import rospy
import termcolor
from jsk_rosbag_tools.bag_to_audio import bag_to_audio


def main():
    parser = argparse.ArgumentParser(description='rosbag to audio')
    parser.add_argument('--out', '-o', default=None,
                        help='output filename. '
                        'If `--audio-topic`_info is exists, '
                        "you don't have to specify samplerate and channels.")
    parser.add_argument('--samplerate', '-r', type=int, help='sampling rate',
                        default='16000')
    parser.add_argument('--channels',
                        type=int, default=1, help='number of input channels')
    parser.add_argument('--audio-topic', type=str, default=None)
    parser.add_argument('input_bagfile')
    parser.add_argument('--start', '-s', default=0, type=float)
    parser.add_argument('--end', '-e', default=sys.maxsize, type=float)
    args = parser.parse_args()

    start_stamp = rospy.Time(args.start)
    end_stamp = rospy.Time(args.end)

    audio_exists = bag_to_audio(
        args.input_bagfile, args.out,
        start_stamp=start_stamp,
        end_stamp=end_stamp,
        samplerate=args.samplerate,
        channels=args.channels,
        topic_name=args.audio_topic)

    if audio_exists is False:
        termcolor.cprint('=> Error exporting audio', 'red')
    else:
        termcolor.cprint('=> Audio files saved', 'green')


if __name__ == '__main__':
    main()
