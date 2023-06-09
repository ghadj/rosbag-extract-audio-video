import datetime
import os
import os.path as osp
import sys
import tempfile
from itertools import chain

import cv2
import rospy

from jsk_rosbag_tools.bag_to_audio import bag_to_audio
from jsk_rosbag_tools.extract import extract_image_topic, get_image_topic_names
from jsk_rosbag_tools.image_utils import \
    resize_keeping_aspect_ratio_wrt_target_size
from jsk_rosbag_tools.info import get_topic_dict
from jsk_rosbag_tools.makedirs import makedirs
from jsk_rosbag_tools.topic_name_utils import topic_name_to_file_name

from moviepy.audio.io.AudioFileClip import AudioFileClip
from moviepy.video.io.ffmpeg_writer import FFMPEG_VideoWriter
from moviepy.video.io.VideoFileClip import VideoFileClip

from tqdm import tqdm


def bag_to_video(image_bagfiles,
                 audio_bagfile=None,
                 output_filepath=None,
                 output_dirpath=None,
                 image_topic=None,
                 image_topics=None,
                 fps=30,
                 start_stamp=rospy.Time(0),
                 end_stamp=rospy.Time(sys.maxsize),
                 samplerate=16000,
                 channels=1,
                 audio_topic='/audio',
                 show_progress_bar=True):
    """Create video from rosbag file.

    Specify only either output_filepath or output_dirpath.
    If output_filepath is specified, specify image_topic.
    If output_dirpath is specified, image_topics can be specified.
    If image_topic_names is None, make all color images into video.

    """
    if len(image_bagfiles) == 0:
        print('[bag_to_video] No bagfiles given.')
        sys.exit(1)

    for image_bagfile_i in image_bagfiles:
        if not os.path.exists(image_bagfile_i):
            print('[bag_to_video] Input bagfile {} not exists.'
                  .format(image_bagfile_i))
            sys.exit(1)

    if output_filepath is not None and output_dirpath is not None:
        raise ValueError(
            'Specify only either output_filepath or output_dirpath.')

    output_filepaths = []
    target_image_topics = []
    candidate_topics = get_image_topic_names(image_bagfiles[0])

    if output_filepath is not None:
        if image_topic is None:
            raise ValueError(
                'If output_filepath is specified, specify image_topic.')
        output_filepaths.append(output_filepath)
        target_image_topics.append(image_topic)
        wav_outpath = tempfile.NamedTemporaryFile(suffix='.wav').name
    else:
        # output_dirpath is specified case.
        if image_topics is None:
            # record all color topics
            image_topics = get_image_topic_names(image_bagfiles[0],
                                                 rgb_only=True)
        target_image_topics = image_topics

        for image_topic in target_image_topics:
            output_filepaths.append(
                osp.join(
                    output_dirpath, '{}_{}.mp4'.format(
                        osp.basename(image_bagfiles[0]).split('.bag')[0], topic_name_to_file_name(image_topic))))

        if audio_bagfile is not None:
            wav_outpath = osp.join(output_dirpath, '{}_{}.wav'.format(
                osp.basename(audio_bagfile).split('.bag')[0], topic_name_to_file_name(audio_topic)))

    # check topics exist.
    not_exists_topics = list(filter(
        lambda tn: tn not in candidate_topics, target_image_topics))
    if len(not_exists_topics) > 0:
        raise ValueError(
            'Topics that are not included in the rosbag are specified.'
            ' {}'.format(list(not_exists_topics)))

    print('[bag_to_video] Extracting audio from rosbag file.')

    # If audio bagfile is given check if audio topic exists
    if audio_bagfile is None:
        audio_exists = False
    else:
        audio_exists = bag_to_audio(audio_bagfile, wav_outpath,
                                    start_stamp=start_stamp,
                                    end_stamp=end_stamp,
                                    samplerate=samplerate,
                                    channels=channels,
                                    topic_name=audio_topic)

    dt = 1.0 / fps
    for image_topic, output_filepath in zip(target_image_topics,
                                            output_filepaths):
        print('[bag_to_video] Creating video of {} from rosbag files {}.'
              .format(image_topic, image_bagfiles))
        filepath_dir = osp.dirname(output_filepath)
        if filepath_dir:
            makedirs(filepath_dir)
        if audio_exists:
            tmp_videopath = '{}_image-only.mp4'.format(
                output_filepath.split('.mp4')[0])
            # tmp_videopath = tempfile.NamedTemporaryFile(suffix='.mp4').name
        else:
            tmp_videopath = output_filepath

        # initialize empty iterable for the images of all bagfiles
        images = iter([])

        # initialize total number of frames of all bagfiles
        n_frame = 0
        
        for image_bagfile_i in image_bagfiles:
            images_i = extract_image_topic(
                image_bagfile_i, image_topic, start_stamp, end_stamp)
            topic_info_dict = get_topic_dict(image_bagfile_i)[image_topic]

            # TODO shows total number of messages, not selected frames
            n_frame_i = topic_info_dict['messages']

            images = chain(images, images_i)
            n_frame = n_frame + n_frame_i

        if show_progress_bar:
            progress = tqdm(total=n_frame)

        # remove 0 time stamp
        stamp = 0.0
        try:
            while stamp == 0.0:
                stamp, _, img, _ = next(images)
                if show_progress_bar:
                    progress.update(1)
        except StopIteration:
            print('[bag_to_video] No timestamp found in all images')
            print('[bag_to_video] Skipping {}'.format(image_topic))
            break
        first_stamp = stamp
        width, height = img.shape[1], img.shape[0]

        creation_time = datetime.datetime.utcfromtimestamp(first_stamp)
        time_format = '%y-%m-%d %h:%M:%S'
        writer = FFMPEG_VideoWriter(
            tmp_videopath,
            (width, height),
            fps, logfile=None,
            ffmpeg_params=[
                '-metadata',
                'creation_time={}'.format(
                    creation_time.strftime(time_format)),
            ])

        current_time = 0.0
        cur_img = resize_keeping_aspect_ratio_wrt_target_size(
            cv2.cvtColor(img, cv2.COLOR_BGR2RGB), width=width, height=height)
        for i, (stamp, _, bgr_img, _) in enumerate(images):
            if show_progress_bar:
                progress.update(1)
            aligned_stamp = stamp - first_stamp
            # write the current image until the next frame's timestamp.
            while current_time < aligned_stamp:
                current_time += dt
                writer.write_frame(cur_img)
            # set current image.
            cur_img = resize_keeping_aspect_ratio_wrt_target_size(
                cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB),
                width=width, height=height)
        writer.close()

        if show_progress_bar:
            progress.close()

        if audio_exists:
            print('[bag_to_video] Combine video and audio')
            clip_output = VideoFileClip(tmp_videopath).subclip().\
                set_audio(AudioFileClip(wav_outpath))
            clip_output.write_videofile(
                output_filepath,
                codec="libx264",
                verbose=False,
                logger='bar' if show_progress_bar else None)
        print('[bag_to_video] Created video is saved to {}'
              .format(output_filepath))
