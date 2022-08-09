#pragma once

#include <optional>
#include <string>

#include <librealsense2/rs.hpp>
#include <rstream/device.h>
#include <spdlog/spdlog.h>


namespace rstream {

spdlog::level::level_enum parseLevel(const std::string& text);

std::string toString(rs2_stream stream, int index);

std::optional<std::pair<rs2_stream, int>>  parseStreamType(const std::string& text);

std::optional<rs2_format> parseStreamFormat(const std::string& text);

}  // namespace rstream
