#include <rstream/frameset.h>

#include <rstream/convert.h>

namespace rstream {

void Frameset::addFrame(rs2::frame frame) {
    auto profile = frame.get_profile();
    auto stream = profile.stream_type();
    auto index = profile.stream_index();

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

cv::Mat Frameset::getMat(rs2_stream stream, int index) {
    cv::Mat mat;
    if (getMat(mat, stream, index)) {
        return mat;
    }

    return {};
}

bool Frameset::getMat(cv::Mat& mat, rs2_stream stream, int index) {
    auto frame = get(stream, index);
    if (!frame) {
        return false;
    }

    return toMat(*frame, mat);
}

}  // namespace rstream
