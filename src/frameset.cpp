#include <rstream/frameset.h>

namespace rstream {

std::shared_ptr<rs2::frame> Frameset::get(rs2_stream stream, int index) {
    auto stream_it = frames_.find(stream);
    if (stream_it == frames_.end()) {
        return {};
    }

    auto index_it = stream_it->second.find(index);
    if (index_it == stream_it->second.end()) {
        return {};
    }

    return index_it->second;
}

void Frameset::addFrame(rs2::frame frame, rs2_stream stream, int index) {
    if (!contains(stream, index)) {
        size_++;
    }

    frames_[stream][index] = std::make_shared<rs2::frame>(frame);
}

bool Frameset::contains(rs2_stream stream, int index) const {
    auto stream_it = frames_.find(stream);
    if (stream_it == frames_.end()) {
        return false;
    }

    auto index_it = stream_it->second.find(index);
    if (index_it == stream_it->second.end()) {
        return false;
    }

    return true;
}

size_t Frameset::size() const {
    return size_;
}

}  // namespace rstream
