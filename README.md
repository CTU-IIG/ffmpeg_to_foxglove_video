ffmpeg_to_foxglove_video
================

This package contains a tool to convert a ROS bag with ffmpeg-compressed video into a ROS bag that can be replaied directly in Foxglove Studio, without additional tooling. In other words, it converts messages of type `ffmpeg_image_transport_msgs.msg.FFMPEGPacket` to type `foxglove_msgs.msg.CompressedVideo`.

The tool transforms specified [ffmpeg_image_transport][] topics of an input ROS bag into Foxglove video topics in an output ROS bag. The command to convert the ROS bag into new ROS bag with foxglove video is:

```
ros2 run ffmpeg_to_foxglove_video convert path_to_input_rosbag/
```

This is basic usage, where the only converted topic is ZED camera `left` image stream and the output rosbag is named after the input rosbag with suffix `_foxglove`. The user can specify input topics using the `--topic_from` or `--topic-from-regex`, for example:

```
ros2 run ffmpeg_to_foxglove_video convert path_to_input_rosbag/ --topic.from-regex "/ffmpeg$"
```

The command above converts every topic ending with "/ffmpeg". So according to video transport standard it will convert every video stream compressed by ffmpeg.

User can customize output ROS bag using `---output` or `--output-suffix` ("_foxglove"). See `ros2 run ffmpeg_to_foxglove_video convert --help`.

!!! Note: The foxglove_video transport messages do not have the header (`std_msgs/Header`), which means there are no timestamps of each frame. This renders foxglove_video transport useless as default compression method, because we wouldn't be able to process video based on frame timestamp or to restamp it (see **Removing delays of ROS bag messages** in [data/bags/README.md](../../../data/bags/README.md)).

[ffmpeg_to_foxglove_video]: ../../ros2_ws/src/ffmpeg_to_foxglove_video/
[ffmpeg_image_transport]: https://github.com/ros-misc-utilities/ffmpeg_image_transport
