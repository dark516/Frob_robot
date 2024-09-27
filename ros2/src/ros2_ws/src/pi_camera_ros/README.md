# Raspberry Pi Camera ROS2 Publisher

This a simple ROS2 package to publish and view compressed
image frames from a RPI camera.

## Dependencies

You need:

* OpenCV
* `compressed_image_transport`

## Installing

Clone this into `src` of your ROS workspace (e.g `~/ros2_ws/src`):

```
git clone https://github.com/rpapallas/pi_camera_ros
```

Build using `colcon`:

```
cd ~/ros2_ws
colcon build
```

## Running

Run the publisher:

```
ros2 run pi_camera publish_compressed
```

View the compressed frames (possibly on another machine on the same ROS network):

```
ros2 run pi_camera view_compressed
```

## Adjusting parameters

You can adjust the following parameters:

* **Publishing rate:** this is defined in `pi_camera/publish_compressed.py` as
`time_in_seconds` in the initialiser.
* **Frame resolution:** width and height of the frame can be adjusted in
`pi_camera/publish_compressed.py` in the initialiser.
