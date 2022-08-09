#pragma once

#include <unordered_map>

#include <librealsense2/rs.hpp>

namespace rstream {

class Frameset {
  public:
    using Ptr = std::shared_ptr<Frameset>;
    Frameset() = default;
    ~Frameset() = default;

    std::shared_ptr<rs2::frame> get(rs2_stream stream, int index = 0);
    void addFrame(rs2::frame frame, rs2_stream stream, int index = 0);
    bool contains(rs2_stream stream, int index = 0) const;
    size_t size() const;

  private:
    std::unordered_map<rs2_stream, std::unordered_map<int, std::shared_ptr<rs2::frame>>> frames_;
    size_t size_ = 0;
};

}  // namespace rstream
