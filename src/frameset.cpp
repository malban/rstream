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

#include <rstream/frameset.h>

#include <rstream/convert.h>

namespace rstream {

void Frameset::addFrame(rs2::frame frame) {
    auto profile = frame.get_profile();
    StreamIndex stream = { profile.stream_type(), profile.stream_index() };
    if (!contains(stream)) {
        size_++;
    }

    frames_[stream] = std::make_shared<rs2::frame>(frame);
}

bool Frameset::contains(const StreamIndex& stream) const {
    return frames_.count(stream) > 0;
}

size_t Frameset::size() const {
    return size_;
}

std::shared_ptr<rs2::frame> Frameset::get(const StreamIndex& stream) {
    auto stream_it = frames_.find(stream);
    if (stream_it == frames_.end()) {
        return {};
    }
    return stream_it->second;
}

cv::Mat Frameset::getMat(const StreamIndex& stream) {
    cv::Mat mat;
    if (getMat(mat, stream)) {
        return mat;
    }

    return {};
}

bool Frameset::getMat(cv::Mat& mat, const StreamIndex& stream) {
    auto frame = get(stream);
    if (!frame) {
        return false;
    }

    return toMat(*frame, mat);
}

const std::unordered_map<StreamIndex, std::shared_ptr<rs2::frame>>& Frameset::frames() const {
    return frames_;
}

}  // namespace rstream
