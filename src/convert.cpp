#include <rstream/convert.h>

#include <cstring>

#include <spdlog/spdlog.h>

using namespace spdlog;

namespace rstream {

bool toMat(const rs2::frame& frame, cv::Mat& mat) {
    auto profile = frame.get_profile().as<rs2::video_stream_profile>();
    auto format = profile.format();

    size_t mat_size = mat.step[0] * mat.rows;
    int frame_size = frame.get_data_size();
    if (format == RS2_FORMAT_BGR8 || format == RS2_FORMAT_RGB8) {
        if (mat_size != frame_size || mat.cols != profile.width() || mat.rows != profile.height() || mat.type() != CV_8UC3) {
            mat = cv::Mat(profile.height(), profile.width(), CV_8UC3);
            mat_size = mat.step[0] * mat.rows;
            if (mat_size != frame_size) {
                error("cv::Mat has incorrect size: {} != {}", mat_size, frame_size);
                return false;
            }
        }
    }
    else if (format == RS2_FORMAT_BGRA8 || format == RS2_FORMAT_RGBA8) {
        if (mat_size != frame_size || mat.cols != profile.width() || mat.rows != profile.height() || mat.type() != CV_8UC4) {
            mat = cv::Mat(profile.height(), profile.width(), CV_8UC4);
            mat_size = mat.step[0] * mat.rows;
            if (mat_size != frame_size) {
                error("cv::Mat has incorrect size: {} != {}", mat_size, frame_size);
                return false;
            }
        }
    }
    else if (format == RS2_FORMAT_Y8) {
        if (mat_size != frame_size || mat.cols != profile.width() || mat.rows != profile.height() || mat.type() != CV_8UC1) {
            mat = cv::Mat(profile.height(), profile.width(), CV_8UC1);
            mat_size = mat.step[0] * mat.rows;
            if (mat_size != frame_size) {
                error("cv::Mat has incorrect size: {} != {}", mat_size, frame_size);
                return false;
            }
        }
    }
    else if (format == RS2_FORMAT_Y16 || format == RS2_FORMAT_Z16) {
        if (mat_size != frame_size || mat.cols != profile.width() || mat.rows != profile.height() || mat.type() != CV_16UC1) {
            mat = cv::Mat(profile.height(), profile.width(), CV_16UC1);
            mat_size = mat.step[0] * mat.rows;
            if (mat_size != frame_size) {
                error("cv::Mat has incorrect size: {} != {}", mat_size, frame_size);
                return false;
            }
        }
    }
    else {
        warn("Unsupported format for conversion: {}", rs2_format_to_string(format));
        return false;
    }

    std::memcpy(mat.data, frame.get_data(), frame_size);
    return true;
}

}  // namespace rstream
