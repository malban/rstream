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
    if (index == 0) {
        return rs2_stream_to_string(stream);
    }

    return std::string(rs2_stream_to_string(stream)) + "-" +  std::to_string(index);
}

std::optional<std::pair<rs2_stream, int>> parseStreamType(const std::string& text) {
    auto input = boost::to_lower_copy(text);
    if (input == "color") {
        return std::make_pair(RS2_STREAM_COLOR, 0);
    }
    else if (input == "depth") {
        return std::make_pair(RS2_STREAM_DEPTH, 0);
    }
    else if (input == "infra") {
        return std::make_pair(RS2_STREAM_INFRARED, 0);
    }
    else if (input == "infra1") {
        return std::make_pair(RS2_STREAM_INFRARED, 1);
    }
    else if (input == "infra2") {
        return std::make_pair(RS2_STREAM_INFRARED, 2);
    }
    else if (input == "fisheye") {
        return std::make_pair(RS2_STREAM_FISHEYE, 0);
    }
    else if (input == "confidence") {
        return std::make_pair(RS2_STREAM_CONFIDENCE, 0);
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
