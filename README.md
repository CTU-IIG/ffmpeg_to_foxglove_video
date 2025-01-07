ffmpeg_to_foxglove_video
================

This package contains a tool to convert a ROS bag with ffmpeg-compressed video into a ROS bag that can be replayed directly in Foxglove Studio, without additional tooling. In other words, it converts messages of type `ffmpeg_image_transport_msgs.msg.FFMPEGPacket` to type `foxglove_msgs.msg.CompressedVideo`.

The tool transforms specified [ffmpeg_image_transport][] topics of an input ROS bag into Foxglove video topics in an output ROS bag. The command to convert the ROS bag into a new ROS bag with Foxglove video is:

```
ros2 run ffmpeg_to_foxglove_video convert path_to_input_rosbag/
```

This is basic usage, where the only converted topics are ffmpeg camera image streams and the output ROS bag is named after the input ROS bag with the suffix `_foxglove`. The user can specify input topics using the `--from_topic` or `--regex`, for example:

```
ros2 run ffmpeg_to_foxglove_video convert path_to_input_rosbag/ --regex "zed2/zed_node/left/image_rect_color/ffmpeg$"
```

The command above converts every topic ending with `zed2/zed_node/left/image_rect_color/ffmpeg`. It will specifically convert the left camera output video stream compressed by ffmpeg of the ZED2 camera.

Users can customize the output ROS bag path using `---output` or `--output-suffix` (e.g., `_foxglove`), convert the input ROS bag while keep the input ffmpeg video stream using the `-k` flag, or disable printing of the progress using the `-p` flag. See `ros2 run ffmpeg_to_foxglove_video convert --help` for more details.

> [!NOTE]
> The foxglove_video transport messages do not have the header
> (`std_msgs/Header`), which means there are no timestamps of each
> frame. This renders foxglove_video transport useless as default
> compression method, because we wouldn't be able to process video
> based on frame timestamp or to restamp with [ros2bag_tools][].


[ffmpeg_to_foxglove_video]: ../../ros2_ws/src/ffmpeg_to_foxglove_video/
[ffmpeg_image_transport]: https://github.com/ros-misc-utilities/ffmpeg_image_transport
[ros2bag_tools]: https://github.com/AIT-Assistive-Autonomous-Systems/ros2bag_tools
