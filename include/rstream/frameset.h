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

#include <unordered_map>

#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include <rstream/common.h>

namespace rstream {

class Frameset {
  public:
    using Ptr = std::shared_ptr<Frameset>;
    Frameset() = default;
    ~Frameset() = default;

    /**
     * Add a frame to the frameset.
     *
     * Note: Existing data from the same stream,index will be superceded.
     *
     * @param[in]  The frame to add.
     */
    void addFrame(rs2::frame frame);

    /**
     * Check if the frameset contains frame data from a given stream and index.
     *
     * @param[in] stream  Stream type.
     *
     * @returns True if the frame exists, false otherwise.
     */
    bool contains(const StreamIndex& stream) const;

    /**
     * The number of frames in the frameset from distince streams.
     *
     * @returns The number of frames in the frameset from distince streams.
     */
    size_t size() const;

    /**
     * Get the raw frame data from a specific stream.
     *
     * @param[in] stream  Stream type.
     *
     * @returns The raw frame data if it exists, null otherwise.
     */
    std::shared_ptr<rs2::frame> get(const StreamIndex& stream);

    /**
     * Get a cv::Mat copy of frame data from a specific stream.
     *
     * Note:  Only BGR8, BGRA8, RGB8, RGBA8, Y8, Y16, Z16 formats supported.
     *
     * @param[in] stream  Stream type.
     *
     * @returns A cv::Mat copy of the frame if valid, empty otherwise.
     */
    cv::Mat getMat(const StreamIndex& stream);

    /**
     * Copy of frame data into an existing cv::Mat from a specific stream.
     *
     * Note: The cv::Mat will be reallocated if it isn't the correct size and
     *       format.
     *
     * Note:  Only BGR8, BGRA8, RGB8, RGBA8, Y8, Y16, Z16 formats supported.
     *
     * @param[out] mat  Mat to copy frame to.
     * @param[in] stream  Stream type.
     *
     * @returns True if successful, false otherwise.
     */
    bool getMat(cv::Mat& mat, const StreamIndex& stream);

    const std::unordered_map<StreamIndex, std::shared_ptr<rs2::frame>>& frames() const;

  private:
    std::unordered_map<StreamIndex, std::shared_ptr<rs2::frame>> frames_;
    size_t size_ = 0;
};

}  // namespace rstream
