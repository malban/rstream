#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>

namespace rstream {

/**
 * Copy frame data into a cv::Mat.
 *
 * Note: The cv::Mat will be reallocated if it isn't the correct size and
 *       format.
 *
 * Note:  Only BGR8, BGRA8, RGB8, RGBA8, Y8, Y16, Z16 formats supported.
 *
 * @param[in] frame  Frame data.
 * @param[in] mat    Mat to copy to.
 *
 * @returns True if successful, false otherwise.
 */
bool toMat(const rs2::frame& frame, cv::Mat& mat);

}  // namespace rstream
