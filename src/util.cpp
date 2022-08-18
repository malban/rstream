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

#include <rstream/util.h>

#include <utility>

#include <boost/algorithm/string.hpp>

namespace rstream {

spdlog::level::level_enum parseLevel(const std::string& text) {
  auto input = boost::to_lower_copy(text);
  if (text == "trace") {
    return spdlog::level::trace;
  }
  else if (text == "debug") {
    return spdlog::level::debug;
  }
  else if (text == "info") {
    return spdlog::level::info;
  }
  else if (text == "warn") {
    return spdlog::level::warn;
  }
  else if (text == "error") {
    return spdlog::level::err;
  }

  return spdlog::level::off;
}

std::string toString(rs2_stream stream, int index) {
    std::string name = boost::to_lower_copy(std::string(rs2_stream_to_string(stream)));
    if (stream == RS2_STREAM_INFRARED) {
        name = "infra";
    }

    if (index == 0) {
        return name;
    }

    return name + std::to_string(index);
}

std::string toString(const StreamIndex& stream) {
    return toString(stream.stream, stream.index);
}

std::string toString(rs2_format format) {
    return rs2_format_to_string(format);
}

std::optional<StreamIndex> parseStreamType(const std::string& text) {
    auto input = boost::to_lower_copy(text);
    if (input == "color") {
        return StreamIndex({RS2_STREAM_COLOR, 0});
    }
    else if (input == "depth") {
        return StreamIndex({RS2_STREAM_DEPTH, 0});
    }
    else if (input == "infra") {
        return StreamIndex({RS2_STREAM_INFRARED, 0});
    }
    else if (input == "infra1") {
        return StreamIndex({RS2_STREAM_INFRARED, 1});
    }
    else if (input == "infra2") {
        return StreamIndex({RS2_STREAM_INFRARED, 2});
    }
    else if (input == "fisheye") {
        return StreamIndex({RS2_STREAM_FISHEYE, 0});
    }
    else if (input == "confidence") {
        return StreamIndex({RS2_STREAM_CONFIDENCE, 0});
    }

    return {};
}

std::optional<rs2_format> parseStreamFormat(const std::string& text) {
    auto input = boost::to_lower_copy(text);
    if (input == "bgr8") {
        return RS2_FORMAT_BGR8;
    }
    else if (input == "bgra8") {
        return RS2_FORMAT_BGRA8;
    }
    else if (input == "rgb8") {
        return RS2_FORMAT_RGB8;
    }
    else if (input == "rgba8") {
        return RS2_FORMAT_RGBA8;
    }
    else if (input == "raw8") {
        return RS2_FORMAT_RAW8;
    }
    else if (input == "raw10") {
        return RS2_FORMAT_RAW10;
    }
    else if (input == "raw16") {
        return RS2_FORMAT_RAW16;
    }
    else if (input == "uyvy") {
        return RS2_FORMAT_UYVY;
    }
    else if (input == "y8") {
        return RS2_FORMAT_Y8;
    }
    else if (input == "y16") {
        return RS2_FORMAT_Y16;
    }
    else if (input == "yuyv") {
        return RS2_FORMAT_YUYV;
    }
    else if (input == "z16") {
        return RS2_FORMAT_Z16;
    }

    return {};
}

}  // namespace rstream
