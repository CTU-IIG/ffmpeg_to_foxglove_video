'''Program to convert common ffmpeg messages to foxglove studio compressed video messages

author: 2024 Jan Vojnar
'''

from rclpy.serialization import serialize_message, deserialize_message
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
from foxglove_msgs.msg import CompressedVideo
import rosbag2_py
import time
from collections import deque
import sys
import os
import errno
from pathlib import Path
import re

def try_remove_empty_rosbag(path:Path):
    try:
        for file in os.listdir(path):
            print(f"removing {path/file}")
            os.remove(path/file)
        print(f"removing {path}")
        os.rmdir(path)
    except OSError as ex:
        if ex.errno != errno.ENOENT:
            raise
        else:
            print("not exist")

def is_readable(path):
    file_path = Path(path)
    can_read = os.access(file_path, os.R_OK)
    return can_read

def is_writable(path):
    file_path = Path(path)
    can_write = os.access(file_path, os.W_OK)
    return can_write

def is_first_existing_parent_writable(path):
    file_path = Path(path)
    writable = False
    file_path = file_path.expanduser().resolve()
    while (file_path != Path("/")):
        if not file_path.exists():
            file_path = file_path.parent
        else:
            writable = is_writable(file_path)
            break
    return writable

# Foxglove message record:
# print(msg[0],
# deserialize_message(msg[1], CompressedVideo),
# msg[2])
# 
# /sensor_stack/cameras/zed2/zed_node/left/image_rect_color/foxglove
# foxglove_msgs.msg.CompressedVideo(
#       timestamp=builtin_interfaces.msg.Time(sec=1727787341, nanosec=829349749),
#       frame_id='zed2_left_camera_optical_frame',
#       data=[0, ..., 64],
#       format='h264')
# 1727968225678140596


# Foxglove topic metadata:
# print(topic.name,
# topic.offered_qos_profiles,
# topic.serialization_format,
# topic.type)
# 
# /sensor_stack/cameras/zed2/zed_node/left/image_rect_color/foxglove
#
# history: 3
# depth: 0
# reliability: 1
# durability: 2
# liveliness: 1
# avoid_ros_namespace_conventions: false
# 
# cdr
# 
# foxglove_msgs/msg/CompressedVideo

class App():
    ERROR = 1
    OK = 0

    def __init__(self, args):
        self.args = args

    def run(self):
        regex = None
        #check topic_from ff typed in, then do not save default regex
        #else if regex is typed in, then use it
        if self.args.from_topic is not None:
            topic_from = self.args.from_topic
            # we assume that user behaves normally and topic_from is ffmpeg named subpath of some video topic
            # with ffmpeg compression method so parent of topic_from should be topic name of the video topic
            topic_to = topic_from[:topic_from.rfind('/')]+'/foxglove'
            if self.args.to_topic != '':
                topic_to = self.args.to_topic
        else:
            try:
                regex = re.compile(self.args.regex)
            except re.error as ex:
                print(f'The input regex is not a valid regular expression ("{self.args.regex}").', file=sys.stderr)
                raise

        input_path = Path(self.args.input).expanduser().resolve()
        # rosbag2 reader do not handle bad rosbag path well, we should check for user errors
        if not input_path.exists():
            print(f'The input rosbag does not exist on the path ("{input_path}").', file=sys.stderr)
            return App.ERROR
        if not input_path.is_dir():
            print(f'A file is selected as input rosbag but directory is needed ("{input_path}").', file=sys.stderr)
            return App.ERROR
        if not is_readable(input_path):
            print(f'A rosbag does not exist on the path or it is not ROS2 bag ("{input_path}").', file=sys.stderr)
            return App.ERROR
        try:
            print(f"Input rosbag path: {input_path}")
            self.reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py._storage.StorageOptions(
                uri=str(input_path),
                storage_id='mcap')
            converter_options = rosbag2_py._storage.ConverterOptions('', '')
            self.reader.open(storage_options, converter_options)
        except Exception as ex:
            raise

        output_path = input_path
        # rosbag2 writer do not handle bad rosbag path well and can cause side-effects
        # like creating empty database or writing to existing database
        # we should check the path for user errors
        if self.args.output != '':
            output_path = Path(self.args.output).expanduser().resolve()
        output_path = output_path.parent/Path(f"{output_path.name}{self.args.suffix}")
        # If Path exists
        if output_path.exists():
            #and If it is directory, is it empty?
            if output_path.is_dir():
                if len(list(output_path.iterdir())) > 0:
                    print(f'Output directory exists but it is not empty ("{output_path}").', file=sys.stderr)
                    return App.ERROR
        # Can we create the directory and files in it?
        is_writable = is_first_existing_parent_writable(output_path)
        if not is_writable:
            print(f'The user do not have permissions to write the path ("{output_path}").', file=sys.stderr)
            return App.ERROR
        try:
            print(f"Output rosbag path: {output_path}")
            self.writer = rosbag2_py.SequentialWriter()
            storage_options = rosbag2_py._storage.StorageOptions(
                uri=str(output_path),
                storage_id='mcap')
            converter_options = rosbag2_py._storage.ConverterOptions('', '')
            self.writer.open(storage_options, converter_options)
        except Exception as ex:
            # sadly, the self.writer most probably created an empty rosbag at the path
            # already handled 'path exists and not empty' ensures that we do not remove some other rosbag
            try_remove_empty_rosbag(output_path)
            raise

        msg_count = 0
        old_metadata = self.reader.get_metadata()
        old_topics = old_metadata.topics_with_message_count

        #regex data
        matched_topics = {}

        for topic_info in old_topics:
            topic = topic_info.topic_metadata
            if regex is not None:
                if (regex.search(topic.name)):
                    old_topic_path = topic.name
                    new_topic_path = old_topic_path[:old_topic_path.rfind('/')]+'/foxglove'
                    matched_topics[old_topic_path] = new_topic_path
                    new_topic = rosbag2_py.TopicMetadata(
                        id = topic.id,
                        name = str(new_topic_path),
                        serialization_format = 'cdr',
                        type = 'foxglove_msgs/msg/CompressedVideo')
                    self.writer.create_topic(new_topic)
                    msg_count += topic_info.message_count
                    print(f'Target topic found: {old_topic_path}')
                    if self.args.keep_input:
                        self.writer.create_topic(topic)
                else:
                    self.writer.create_topic(topic)
            else:
                if (topic.name == topic_from):
                    matched_topics[topic_from] = topic_to
                    new_topic = rosbag2_py.TopicMetadata(
                        id = topic.id,
                        name = str(topic_to),
                        serialization_format = 'cdr',
                        type = 'foxglove_msgs/msg/CompressedVideo')
                    self.writer.create_topic(new_topic)
                    msg_count = topic_info.message_count
                    print('Target topic found.')
                    if self.args.keep_input:
                        self.writer.create_topic(topic)
                else:
                    self.writer.create_topic(topic)
        
        if msg_count == 0:
            print(f'Input topic(s) does not have messages or does not exist.', file=sys.stderr)
            print(f'Nothing to process.')
            # sadly, the self.writer already created an empty rosbag at the path
            # already handled 'path exists and not empty' ensures that we do not remove some other rosbag
            try_remove_empty_rosbag(output_path)
            return 1
        
        time_start = time.time()
        time_per_single_window = deque(maxlen=int(msg_count/10))
        progress = 0.01
        progress_msg = '{:3.0f} %'.format((progress*100)/msg_count)
        last_progress_msg = progress_msg

        while self.reader.has_next():
            msg = self.reader.read_next()
            new_topic_path = matched_topics.get(msg[0], None)
            if new_topic_path is not None:
                old_data = deserialize_message(msg[1], FFMPEGPacket)
                new_data = CompressedVideo(
                    timestamp=old_data.header.stamp,
                    frame_id = old_data.header.frame_id,
                    data = old_data.data,
                    format='h264')
                self.writer.write(
                    new_topic_path,
                    serialize_message(new_data), msg[2])
                if self.args.keep_input:
                    self.writer.write(msg[0], msg[1], msg[2])

                if not self.args.no_progress:
                    progress += 1
                    time_now = time.time()
                    time_per_single_window.append((time_now-time_start)/progress)
                    time_per_single = max(time_per_single_window)
                    progress_perc = (progress*100)/msg_count
                    progress_msg = '{:3.0f} %'.format(progress_perc)
                    if progress_msg != last_progress_msg:
                        last_progress_msg = progress_msg
                        minutes = time_per_single * (msg_count + 1 - progress) // 60
                        seconds = time_per_single * (msg_count + 1 - progress) % 60
                        print(progress_msg + ' - est. remaining time {} {:.0f}:{:02.0f}'.format(
                            '<' if (minutes == 0 and seconds < 1) else "~",
                            minutes,
                            1 if (minutes == 0 and seconds < 1) else seconds))
            else:
                self.writer.write(msg[0], msg[1], msg[2])
        print("Done")
        return 0

_DESC = \
'''Processes a ROS bag image topic from ffmpeg to foxglove compressed camera video. The mcap storage method is expected.

example: convert /rosbags/input_bag/
To convert input ROS bag into new "/rosbag/input_bag_foxglove/" where every topic ending with "/ffmpeg" is converted to "/foxglove".
'''

def main():
    import argparse
    parser = argparse.ArgumentParser(description=_DESC,
                                     epilog='Note: In case of an error, the output ROS bag may be created but empty.',
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', type=str,
                        help='the input rosbag path')
    group_from = parser.add_argument_group('source topics (mutually exclusive; default is --regex)')#, 'Source topics from which the video will be converted')
    group_ex = group_from.add_mutually_exclusive_group(required=False)
    group_ex.add_argument('--from-topic', type=str, metavar='TOPIC',
                        help='ffmpeg topic name (example: "/sensors/camera/image/ffmpeg")')
    group_ex.add_argument('--regex', type=str, default='/ffmpeg$',
                        help='regex to filter input topic names (default = "/ffmpeg$")')
    group_to = parser.add_argument_group('destination topics (applies to --from-topic)')#, 'Destination topics to which the video will be converted.')
    group_to.add_argument('--to-topic', type=str, metavar='TOPIC',
                        default='',
                        help='foxglove topic name (example: "/sensors/camera/image/foxglove")',)
    parser.add_argument('--output', type=str, metavar='PATH',
                        default='',
                        help='path of the output rosbag (default: input path)')
    parser.add_argument('--suffix', type=str,
                        default='_foxglove',
                        help='suffix of the output rosbag (default: "_foxglove")')
    parser.add_argument('-p', '--no-progress', action="store_true",
                        help='disable printing of the percentage progress')
    parser.add_argument('-k', '--keep-input', action="store_true",
                        help='keep input topics while adding new output topics')
    args = parser.parse_args()

    app = App(args)
    try:
        exit_code = app.run()
        if exit_code:
            parser.print_usage()
    except Exception as ex:
        # There is no clear documentation what exceptions rosbag2_py or (de)serialization can throw,
        # so we need to find them hard way
        message = f'Programmer\'s note: An exception of type {type(ex).__module__}.{type(ex).__name__} occurred. Arguments:\n{ex.args}'
        print(message)
        raise
    sys.exit(exit_code)
    
if __name__ == '__main__':
    main()