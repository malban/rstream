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

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <librealsense2/rs.hpp>
#include <rstream/frameset.h>

namespace rstream {

class Device {
  public:
    using Ptr = std::shared_ptr<Device>;
    using FrameCallback = std::function<void(Frameset::Ptr)>;

    ~Device();

    void hardwareReset();

    std::string getSerialNo() const;

    /**
     * Configures which streams the device should generate.
     *
     * Note:  If the device is already open, the configuration will only take
     *        effect after it has been closed and reopened.
     *
     * @param[in] config  Stream configurations.
     * @param[in] exact_fps  Require the stream FPS to exactly match the configuration.
     *
     * @returns True if the configurations were valid for this device.
     */
    bool configureStreams(const std::vector<StreamConfig>& config, bool exact_fps=true);

    /**
     * Gets the set of sensor modules associated with the configured streams.
     *
     * @returns The sensor modules associated with the configured streams.
     */
    std::vector<rs2::sensor> getConfiguredSensors();

    /**
     * Gets a stream profile for a given stream.
     *
     * @param[in] stream  The stream index.
     *
     * @returns The corresponding stream profile if it exists, empty otherwise.
     */
    std::optional<rs2::stream_profile> getProfile(const StreamIndex& stream);

    /**
     * Open the configured device streams for exclusive access.
     *
     * Note: This begins the USB data streams and there is a 100ms+ delay before
     *       data starts streaming.
     *
     * @returns True if the device is successfully opened, or is already open.
     */
    bool open();

    /**
     * Close the configured device streams for exclusive access.
     *
     * Note: This stops the USB data streams.
     *
     * @returns True if the device is successfully closed, or is already closed.
     */
    bool close();

    /**
     * Start receiving and processing frames from open device streams.
     *
     * Note: This will begin copying and processing frames received over the
     *       open USB data streams.  There is little to no delay in starting
     *       the processing of already opened streams.
     *
     * @returns True if the device is open and stream processing is successfully
     *          started.
     */
    bool start();

    /**
     * Stop receiving and processing frames from open device streams.
     *
     * Note: This will stop copying and processing frames received over the
     *       open USB data streams.
     *
     * @returns True if stream processing is successfully stopped.
     */
    bool stop();

    /**
     * Set the frame callback function when in streaming mode (not polling with
     * getFrames())
     *
     * @param[in] callback  The callback function to use.
     */
    void setCallback(FrameCallback callback);

    /**
     * Get the next set of frames from the configured device streams.
     *
     * The device streams will be polled by starting them, receiving the first
     * set of frames and then stopping them to avoid the continous processing
     * of frames.
     *
     * Note: The streams cannot already be started prior to calling getFrames()
     *
     * Note: This will open and then close the device streams if they are not
     *       already open, which will incur an additional delay.  If the streams
     *       are already open this can be avoided.
     *
     * @param[in] timeout_ms  Timeout to wait for frames in ms.
     *
     * @returns The next set of frames if successful, null otherwise.
     */
    Frameset::Ptr getFrames(int timeout_ms);

    /**
     * Find and return a device reference with the provided serial number.
     *
     * Note: If the provided serial number is empty, the first device will be
     *       returned.
     *
     * @param[in] serial_no  The serial number to find.
     *
     * @returns The matching device if it is found, null otherwise.
     */
    static Device::Ptr find(const std::string& serial_no);

  private:
    Device(const std::string& serial_no);
    bool connect();
    std::optional<StreamDefinition> configureStream(const StreamConfig& config, bool exact_fps=true);
    void frameCallback(rs2::frame frame);

    std::string serial_no_;
    rs2::context context_;
    rs2::device device_;

    std::string firmware_version_;
    std::string model_;
    std::string physical_port_;
    std::string product_id_;

    std::mutex device_mutex_;
    std::mutex frameset_mutex_;
    std::condition_variable frameset_condition_;
    bool waiting_for_frames_ = false;
    Frameset::Ptr frameset_;

    bool is_open_ = false;
    bool is_streaming_ = false;
    std::unordered_map<std::string, std::vector<StreamDefinition>> sensor_streams_;
    std::vector<rs2::sensor> sensors_;
    size_t stream_num_ = 0;
    FrameCallback frame_callback_;
};

}  // namespace rstream
