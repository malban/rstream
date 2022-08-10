#pragma once

#include <optional>
#include <string>

#include <librealsense2/rs.hpp>
#include <rstream/device.h>
#include <spdlog/spdlog.h>


namespace rstream {

/**
 * Parse log level from string.
 *
 * @param[in] text  Log level string.
 *
 * @returns The log level if the string is valid, spdlog::level::off otherwise.
 */
spdlog::level::level_enum parseLevel(const std::string& text);

/**
 * Parse stream type and index from string.
 *
 * @param[in] text  Stream type string.
 *
 * @returns The stream, index pair if valid, empty otherwise.
 */
std::optional<std::pair<rs2_stream, int>>  parseStreamType(const std::string& text);

/**
 * Parse stream format from string.
 *
 * @param[in] text  Stream format string.
 *
 * @returns The stream format if valid, empty otherwise.
 */
std::optional<rs2_format> parseStreamFormat(const std::string& text);

/**
 * Stringify the provided stream, index pair.
 *
 * @param[in] stream  Steam type.
 * @param[in] index   Stream index.
 *
 * @returns A string label of the stream, index pair.
 */
std::string toString(rs2_stream stream, int index);

}  // namespace rstream
