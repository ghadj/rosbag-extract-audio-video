import os
import os.path as osp
import sys

import numpy as np
import rosbag
import rospy
from scipy.io.wavfile import write as wav_write

from jsk_rosbag_tools.extract import extract_oneshot_topic
from jsk_rosbag_tools.info import get_topic_dict
from jsk_rosbag_tools.makedirs import makedirs


def bag_to_audio(bag_filepath,
                 wav_outpath=None,
                 topic_name=None,
                 audio_info_topic_name=None,
                 start_stamp=rospy.Time(0),
                 end_stamp=rospy.Time(sys.maxsize),
                 samplerate=44100,
                 channels=1,
                 overwrite=True):
    if wav_outpath is None:
        wav_outpath = osp.join(
            osp.dirname(bag_filepath),
            osp.splitext(osp.basename(bag_filepath))[0] + '.wav')

    if os.path.exists(wav_outpath) and overwrite is False:
        raise FileExistsError('{} file already exists.'.format(wav_outpath))
    topic_dict = get_topic_dict(bag_filepath)

    all_audio_topics = []
    if topic_name is None:
        # Get all /audio topics
        for t in topic_dict:
            if t.endswith('/audio'):
                all_audio_topics.append(t)
    else:
        all_audio_topics.append(topic_name)

    for audio_topic in all_audio_topics:
        if audio_topic not in topic_dict:
            return

        audio_info_topic_name = audio_info_topic_name or audio_topic + '_info'
        # if audio_info_topic exists, extract info from it.
        if audio_info_topic_name in topic_dict:
            audio_info = extract_oneshot_topic(
                bag_filepath, audio_info_topic_name)
            if audio_info is not None:
                samplerate = audio_info.sample_rate
                channels = audio_info.channels

        bag = rosbag.Bag(bag_filepath)
        audio_buffer = []
        for _, msg, _ in bag.read_messages(topics=[audio_topic], start_time=start_stamp, end_time=end_stamp):
            if msg._type == 'audio_common_msgs/AudioData':
                buf = np.frombuffer(msg.data, dtype='int16')
                buf = buf.reshape(-1, channels)
                audio_buffer.append(buf)
        audio_buffer = np.concatenate(audio_buffer, axis=0)

        # if more than one topic to be extracted create separate file names
        if len(all_audio_topics) > 1:
            wav_outpath_i = '{}_({}).wav'.format(wav_outpath.split(
                '.wav')[0], audio_topic.replace('/', '_slash_'))

        makedirs(osp.dirname(wav_outpath_i))
        wav_write(wav_outpath_i, rate=samplerate, data=audio_buffer)

        valid = os.stat(wav_outpath_i).st_size != 0
        if valid is False:
            return False

    return True
