#pragma once

#include <unordered_map>

#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>

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
     * @param[in] index  Stream index.
     *
     * @returns True if the frame exists, false otherwise.
     */
    bool contains(rs2_stream stream, int index = 0) const;

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
     * @param[in] index   Stream index.
     *
     * @returns The raw frame data if it exists, null otherwise.
     */
    std::shared_ptr<rs2::frame> get(rs2_stream stream, int index = 0);

    /**
     * Get a cv::Mat copy of frame data from a specific stream.
     *
     * Note:  If the raw frame data is UYVY or YUYV it will be converted to
     *        BGR8
     *
     * @param[in] stream  Stream type.
     * @param[in] index  Stream index.
     *
     * @returns A cv::Mat copy of the frame if valid, empty otherwise.
     */
    cv::Mat getMat(rs2_stream stream, int index = 0);

  private:
    std::unordered_map<rs2_stream, std::unordered_map<int, std::shared_ptr<rs2::frame>>> frames_;
    size_t size_ = 0;
};

}  // namespace rstream