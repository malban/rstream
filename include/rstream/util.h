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

#pragma once

#include <optional>
#include <string>

#include <librealsense2/rs.hpp>
#include <rstream/common.h>

namespace rstream {

/**
 * Parse log level from string.
 *
 * @param[in] text  Log level string.
 *
 * @returns The log level if the string is valid, spdlog::level::off otherwise.
 */
int parseLevel(const std::string& text);

/**
 * Parse stream type and index from string.
 *
 * @param[in] text  Stream type string.
 *
 * @returns The stream, index pair if valid, empty otherwise.
 */
std::optional<StreamIndex>  parseStreamType(const std::string& text);

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

/**
 * Stringify the provided stream, index pair.
 *
 * @param[in] stream  Steam type.
 *
 * @returns A string label of the stream, index pair.
 */
std::string toString(const StreamIndex& stream);


/**
 * Stringify the provided stream format.
 *
 * @param[in] format  Steam format.
 *
 * @returns A string label of the stream format.
 */
std::string toString(rs2_format format);

}  // namespace rstream
