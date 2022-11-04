# ROS Wrapper for Intel® RealSense™ Devices


This repository is based upon the official ROS realsense repository: 
https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy


This package is made to be used with the D455 and ROS 

This version supports Noetic distributions.
LibRealSense2 supported version: v2.50.0 (see realsense2_camera release notes)

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone https://github.com/thijs83/realsense-ros
cd ..
catkin init
catkin config -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
```
To source the workspace
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# Usage Instructions
Start the camera node

To start the camera node in ROS:

roslaunch realsense2_camera rs_camera.launch

This will stream all camera sensors and publish on the appropriate ROS topics.

Other stream resolutions and frame rates can optionally be provided as parameters to the 'rs_camera.launch' file.


## Published Topics

The published topics differ according to the device and parameters. After running the above command with D455 attached, the following list of topics will be available (This is a partial list. For full one type rostopic list):

    /camera/color/camera_info
    /camera/color/image_raw
    /camera/color/metadata
    /camera/depth/camera_info
    /camera/depth/image_rect_raw
    /camera/depth/metadata
    /camera/extrinsics/depth_to_color
    /camera/extrinsics/depth_to_infra1
    /camera/extrinsics/depth_to_infra2
    /camera/infra1/camera_info
    /camera/infra1/image_rect_raw
    /camera/infra2/camera_info
    /camera/infra2/image_rect_raw
    /camera/gyro/imu_info
    /camera/gyro/metadata
    /camera/gyro/sample
    /camera/accel/imu_info
    /camera/accel/metadata
    /camera/accel/sample
    /diagnostics



The following parameters are available by the wrapper:

    serial_no: will attach to the device with the given serial number (serial_no) number. Default, attach to available RealSense device in random.

    usb_port_id: will attach to the device with the given USB port (usb_port_id). i.e 4-1, 4-2 etc. Default, ignore USB port when choosing a device.

    device_type: will attach to a device whose name includes the given device_type regular expression pattern. Default, ignore device type. For example, device_type:=d435 will match d435 and d435i. device_type=d435(?!i) will match d435 but not d435i.

    rosbag_filename: Will publish topics from rosbag file.

    initial_reset: On occasions the device was not closed properly and due to firmware issues needs to reset. If set to true, the device will reset prior to usage.

    reconnect_timeout: When the driver cannot connect to the device try to reconnect after this timeout (in seconds).

    align_depth: If set to true, will publish additional topics for the "aligned depth to color" image.: /camera/aligned_depth_to_color/image_raw, /camera/aligned_depth_to_color/camera_info.
    The pointcloud, if enabled, will be built based on the aligned_depth_to_color image.

    filters: any of the following options, separated by commas:

    colorizer: will color the depth image. On the depth topic an RGB image will be published, instead of the 16bit depth values .

    pointcloud: will add a pointcloud topic /camera/depth/color/points.
        The texture of the pointcloud can be modified in rqt_reconfigure (see below) or using the parameters: pointcloud_texture_stream and pointcloud_texture_index. Run rqt_reconfigure to see available values for these parameters.
        The depth FOV and the texture FOV are not similar. By default, pointcloud is limited to the section of depth containing the texture. You can have a full depth to pointcloud, coloring the regions beyond the texture with zeros, by setting allow_no_texture_points to true.
        pointcloud is of an unordered format by default. This can be changed by setting ordered_pc to true.

    hdr_merge: Allows depth image to be created by merging the information from 2 consecutive frames, taken with different exposure and gain values. The way to set exposure and gain values for each sequence in runtime is by first selecting the sequence id, using rqt_reconfigure stereo_module/sequence_id parameter and then modifying the stereo_module/gain, and stereo_module/exposure.
    To view the effect on the infrared image for each sequence id use the sequence_id_filter/sequence_id parameter.
    To initialize these parameters in start time use the following parameters:
    stereo_module/exposure/1, stereo_module/gain/1, stereo_module/exposure/2, stereo_module/gain/2
    * For in-depth review of the subject please read the accompanying white paper.

    The following filters have detailed descriptions in : https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
        disparity - convert depth to disparity before applying other filters and back.
        spatial - filter the depth image spatially.
        temporal - filter the depth image temporally.
        hole_filling - apply hole-filling filter.
        decimation - reduces depth scene complexity.

    enable_sync: gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag. This happens automatically when such filters as pointcloud are enabled.

    <stream_type>_width, <stream_type>_height, <stream_type>_fps: <stream_type> can be any of infra, color, fisheye, depth, gyro, accel, pose, confidence. Sets the required format of the device. If the specified combination of parameters is not available by the device, the stream will be replaced with the default for that stream. Setting a value to 0, will choose the first format in the inner list. (i.e. consistent between runs but not defined).
    *Note: for gyro accel and pose, only _fps option is meaningful.

    enable_<stream_name>: Choose whether to enable a specified stream or not. Default is true for images and false for orientation streams. <stream_name> can be any of infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose, confidence.

    tf_prefix: By default all frame's ids have the same prefix - camera_. This allows changing it per camera.

    <stream_name>_frame_id, <stream_name>_optical_frame_id, aligned_depth_to_<stream_name>_frame_id: Specify the different frame_id for the different frames. Especially important when using multiple cameras.

    base_frame_id: defines the frame_id all static transformations refers to.

    odom_frame_id: defines the origin coordinate system in ROS convention (X-Forward, Y-Left, Z-Up). pose topic defines the pose relative to that system.

    All the rest of the frame_ids can be found in the template launch file: nodelet.launch.xml

    unite_imu_method: The D455 camera have built in IMU components which produce 2 unrelated streams: gyro - which shows angular velocity and accel which shows linear acceleration. Each with it's own frequency. By default, 2 corresponding topics are available, each with only the relevant fields of the message sensor_msgs::Imu are filled out. Setting unite_imu_method creates a new topic, imu, that replaces the default gyro and accel topics. The imu topic is published at the rate of the gyro. All the fields of the Imu message under the imu topic are filled out.
        linear_interpolation: Every gyro message is attached by the an accel message interpolated to the gyro's timestamp.
        copy: Every gyro message is attached by the last accel message.

    clip_distance: remove from the depth image all values above a given value (meters). Disable by giving negative value (default)

    linear_accel_cov, angular_velocity_cov: sets the variance given to the Imu readings. For the T265, these values are being modified by the inner confidence value.

    hold_back_imu_for_frames: Images processing takes time. Therefor there is a time gap between the moment the image arrives at the wrapper and the moment the image is published to the ROS environment. During this time, Imu messages keep on arriving and a situation is created where an image with earlier timestamp is published after Imu message with later timestamp. If that is a problem, setting hold_back_imu_for_frames to true will hold the Imu messages back while processing the images and then publish them all in a burst, thus keeping the order of publication as the order of arrival. Note that in either case, the timestamp in each message's header reflects the time of it's origin.

    calib_odom_file: For the T265 to include odometry input, it must be given a configuration file. Explanations can be found here. The calibration is done in ROS coordinates system.

    publish_tf: boolean, publish or not TF at all. Defaults to True.

    tf_publish_rate: double, positive values mean dynamic transform publication with specified rate, all other values mean static transform publication. Defaults to 0

    publish_odom_tf: If True (default) publish TF from odom_frame to pose_frame.

    infra_rgb: When set to True (default: False), it configures the infrared camera to stream in RGB (color) mode, thus enabling the use of a RGB image in the same frame as the depth image, potentially avoiding frame transformation related errors. When this feature is required, you are additionally required to also enable enable_infra:=true for the infrared stream to be enabled.
        NOTE The configuration required for enable_infra is independent of enable_depth
        NOTE To enable the Infrared stream, you should enable enable_infra:=true NOT enable_infra1:=true nor enable_infra2:=true
        NOTE This feature is only supported by Realsense sensors with RGB streams available from the infra cameras, which can be checked by observing the output of rs-enumerate-devices

