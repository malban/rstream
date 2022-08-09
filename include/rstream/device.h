#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <librealsense2/rs.hpp>
#include <rstream/frameset.h>

namespace rstream {

struct StreamConfig {
    rs2_stream stream;
    int index = 0;
    int width;
    int height;
    int fps;
    rs2_format format;
};

struct StreamDefinition {
    rs2_stream stream;
    int index;
    rs2::sensor sensor;
    rs2::video_stream_profile profile;
};

class Device {
  public:
    using Ptr = std::shared_ptr<Device>;
    ~Device();

    bool configureStreams(const std::vector<StreamConfig>& config);
    bool openStreams();
    bool closeStreams();
    bool startStreams();
    bool stopStreams();
    Frameset::Ptr getFrames(int timeout_ms);

    static Device::Ptr find(const std::string& serial_no);

  private:
    Device(const std::string& serial_no);
    bool connect();
    void frameCallback(rs2::frame frame);
    std::optional<StreamDefinition> configureStream(const StreamConfig& config);

    std::string serial_no_;
    rs2::context context_;
    rs2::device device_;

    std::string firmware_version_;
    std::string model_;
    std::string physical_port_;
    std::string product_id_;

    std::mutex frameset_mutex_;
    std::condition_variable frameset_condition_;
    bool waiting_for_frames_ = false;
    Frameset::Ptr frameset_;

    bool is_open_ = false;
    bool is_streaming_ = false;
    std::unordered_map<std::string, std::vector<StreamDefinition>> sensor_streams_;
    size_t stream_num_ = 0;
};

}  // namespace rstream
