#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>

namespace rstream {

cv::Mat toMat(const rs2::frame& frame);

}  // namespace rstream
