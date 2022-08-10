# rstream
Library for managing multiple Intel RealSense camera streams and minimizing processing.

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
