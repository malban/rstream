/**
 * Copyright (c) 2022, Hatchbed
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
