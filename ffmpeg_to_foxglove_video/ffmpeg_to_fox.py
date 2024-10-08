'''Program to convert common ffmpeg messages to foxglove studio compressed video messages

Copyright (c) 2024 Jan Vojnar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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

def try_remove_empty_rosbag(path):
    try:
        for file in os.listdir(path):
            print(f"removing {path}/{file}")
            os.remove(path+"/"+file)
        print(f"removing {path}")
        os.rmdir(path)
    except OSError as ex:
        if ex.errno != errno.ENOENT:
            raise
        else:
            print("not exist")

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
    def __init__(self, args):
        self.args = args

    def run(self):

        try:
            input_path = self.args.rosbag
            if str.endswith(input_path, ".mcap") or str.endswith(input_path, "metadata.yaml"):
                input_path = input_path[:str.rindex(input_path, '/')]
            print(f"Input rosbag path: {input_path}")
            self.reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py._storage.StorageOptions(
                uri=input_path,
                storage_id='mcap')
            converter_options = rosbag2_py._storage.ConverterOptions('', '')
            self.reader.open(storage_options, converter_options)
        except RuntimeError as ex:
            if str.startswith(ex.args[0], 'No storage could be initialized from the inputs'):
                print(f'A rosbag does not exist on the path or it is not ROS2 bag ("{input_path}").', file=sys.stderr)
                return 1
        except Exception as ex:
            raise

        try:
            output_path = input_path
            if self.args.output in ['', '.', '..', '/', './', '../']:
                if self.args.output:
                    output_path = str(self.args.output).rstrip('/')
                    + '/'
                    + str(input_path).lstrip('./')
                else:
                    output_path = str(input_path).lstrip('./')
            elif self.args.output not in ['', '.', '..', '/', './', '../']:
                output_path = self.args.output

            output_path += str(self.args.output_suffix)
            print(f"Output rosbag path: {output_path}")
            self.writer = rosbag2_py.SequentialWriter()
            storage_options = rosbag2_py._storage.StorageOptions(
                uri=output_path,
                storage_id='mcap')
            converter_options = rosbag2_py._storage.ConverterOptions('', '')
            self.writer.open(storage_options, converter_options)
        except RuntimeError as ex:
            if str.startswith(ex.args[0], 'Database directory already exists'):
                print(f'Output file already exists ("{output_path}").', file=sys.stderr)
                return 1
            elif str.startswith(ex.args[0], 'Failed to create database directory'):
                print(f'Failed to create output directory ("{output_path}"). Do you have rights for this?', file=sys.stderr)
                return 1
            # sadly, the self.writer most probably created an empty rosbag at the path
            # already handled 'path exists' ensures that we do not remove some other rosbag
            try_remove_empty_rosbag(output_path)
            raise
        except Exception as ex:
            # sadly, the self.writer most probably created an empty rosbag at the path
            # already handled 'path exists' ensures that we do not remove some other rosbag
            try_remove_empty_rosbag(output_path)
            raise

        msg_count = 0
        old_metadata = self.reader.get_metadata()
        old_topics = old_metadata.topics_with_message_count
        for topic_info in old_topics:
            topic = topic_info.topic_metadata
            if (topic.name == self.args.topic_from):
                new_topic = rosbag2_py.TopicMetadata(
                    name = self.args.topic_to,
                    serialization_format = 'cdr',
                    type = 'foxglove_msgs/msg/CompressedVideo')
                self.writer.create_topic(new_topic)
                msg_count = topic_info.message_count
                print('Target topic found.')
            else:
                self.writer.create_topic(topic)
        
        if msg_count == 0:
            print(f'Input topic does not have messages or does not exist ("{self.args.topic_from}").', file=sys.stderr)
            print(f'Nothing to process.')
            # sadly, the self.writer already created an empty rosbag at the path
            # already handled 'path exists' ensures that we do not remove some other rosbag
            try_remove_empty_rosbag(output_path)
            return 1
        
        time_start = time.time()
        time_per_single_window = deque(maxlen=int(msg_count/10))
        progress = 0.01
        progress_msg = '{:3.0f} %'.format((progress*100)/msg_count)
        last_progress_msg = progress_msg

        while self.reader.has_next():
            msg = self.reader.read_next()
            if (msg[0] == self.args.topic_from):
                old_data = deserialize_message(msg[1], FFMPEGPacket)
                new_data = CompressedVideo(
                    timestamp=old_data.header.stamp,
                    frame_id = old_data.header.frame_id,
                    data = old_data.data,
                    format='h264')
                self.writer.write(
                    self.args.topic_to,
                    serialize_message(new_data), msg[2])

                progress += 1
                time_now = time.time()
                time_per_single_window.append((time_now-time_start)/progress)
                time_per_single = max(time_per_single_window)
                progress_perc = (progress*100)/msg_count
                progress_msg = '{:3.0f} %'.format(progress_perc)
                if progress_msg != last_progress_msg:
                    last_progress_msg = progress_msg
                    print(progress_msg + ' - est. remaining time {:.0f}:{:02.0f}'.format(
                        time_per_single * (msg_count + 1 - progress) // 60,
                        time_per_single * (msg_count + 1 - progress) % 60))
            else:
                self.writer.write(msg[0], msg[1], msg[2])
        print("Done")
        return 0

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Processes rosbag image topic from ffmpeg to foxglove compressed camera video. The mcap storage method is expected.')
    parser.add_argument('rosbag', type=str,
                        help='Input rosbag path')
    parser.add_argument('--topic-from', type=str,
                        default='/sensor_stack/cameras/zed2/zed_node/left/image_rect_color/ffmpeg',
                        help='FFMPEG topic name (default: "/sensor_stack/cameras/zed2/zed_node/left/image_rect_color/ffmpeg")')
    parser.add_argument('--topic-to', type=str,
                        default='/sensor_stack/cameras/zed2/zed_node/left/image_rect_color/foxglove',
                        help='Foxglove topic name (default: "/sensor_stack/cameras/zed2/zed_node/left/image_rect_color/foxglove")')
    parser.add_argument('--output', type=str,
                        default='',
                        help='Path of the output rosbag, will use input rosbag path if empty')
    parser.add_argument('--output-suffix', type=str,
                        default='_foxglove',
                        help='Suffix of the output rosbag')
    args = parser.parse_args()

    # we strip '/' at the end of paths to prevent issues and inconsistencies
    args.rosbag = args.rosbag.rstrip('/')
    args.output = args.output.rstrip('/')
    
    app = App(args)
    try:
        exit_code = app.run()
        if exit_code:
            parser.print_usage()
    except Exception as ex:
        # There is no clear documentation what exceptions rosbag2_py or (de)serialization can throw,
        # so we need to find them hard way
        message = f'Programmer\'s note: A new exception of type {type(ex).__module__}.{type(ex).__name__} occurred, please implement handling. Arguments:\n{ex.args}'
        print(message)
        raise
    sys.exit(exit_code)
    
if __name__ == '__main__':
    main()