# rstream
Library for managing multiple Intel RealSense camera streams and minimizing processing.

## Motivation

Implement a slightly higher level interface for accessing RealSense devices programmatically than is provided with
librealsense.  The [realsense_ros](https://github.com/IntelRealSense/realsense-ros) package also provides a high level
interface, but it is geared to publishing a fixed rate of image messages from a [ROS](http://wiki.ros.org/) node and is
generally tied to ROS.

Sensor data can be acquired in 3 ways with rstream:
1. Conventional streaming with a callback.

   Data is returned from the sensor at a fixed rate to a callback function.

2. Polling for a single frame from an open stream.

   Sensor streams are opened for exclusive access, but the data sent over the USB interface is not copied or processed
   until a blocking call to get then next frame is made.  This method is useful when a continous stream of data is not
   required for the application and CPU resources are constrained.   This avoids both memory copies and color
   conversions from being performed on data that is not polled for.

   Note: Since the data is being continously streamed on the USB interface, polling for the frame doesn't add any extra
   delay over the conventional streaming/callback method.

3. Polling for a single frame from a closed stream.

   Sensor streams are not opened for exclusive access until a blocking call to get then next frame is made.  Once the 
   frame is returned the streams are closed for exclusive access.  This is useful if the USB interface bandwidth is
   constrained, but adds additional delay of 150ms+ to open the streams.

## Build Dependencies

- [boost](https://www.boost.org/)
- [librealsense](https://github.com/IntelRealSense/librealsense)
- [opencv](https://opencv.org/) for some datatypes

## Example Usage:
```
#include <string>
#include <vector>

#include <rstream/device.h>

int main(int argc, char **argv) {

  std::string serial_no = argv[1];
  auto device = rstream::Device::find(serial_no);
  if (!device) {
    return 1;
  }

  std::vector<rstream::StreamConfig> config;
  config.push_back({
    RS2_STREAM_DEPTH,  // stream
    0,                 // index
    1280,              // width
    720,               // height
    30,                // fps,
    RS2_FORMAT_Z16});  // format

  if (!device->configureStreams(config)) {
    return 1;
  }

  if (!device->open()) {
    return 1;
  }

  int timeout_ms = 1000;
  auto frames = device->getFrames(timeout_ms);

  return 0;
}

```

## CLI Tool

The `rstream_cli` is a test tool that provides a more complete example of using the API.

```
rstream_cli
Usage: ./devel/lib/rstream_cli [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -n,--serial-no TEXT ...     Serial number
  -s,--stream TEXT ...        Stream configuration (TYPE:WIDTH:HEIGHT:FPS:FORMAT)
  -l,--level TEXT             Log level  {trace,debug,info,warn,error,off}
  -m,--mode TEXT:=continuous  Mode  {continuous,user}
  -k,--keep-open              Keep streams open

```

## ROS Support

This library is agnostic to ROS, but is packaged to work in a ROS1 or ROS2
workspace.
