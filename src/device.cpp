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

#include <rstream/device.h>

#include <chrono>

#include <rstream/util.h>
#include <spdlog/spdlog.h>

using namespace std::chrono_literals;
using namespace spdlog;

namespace rstream {

Device::Device(const std::string& serial_no) : serial_no_(serial_no) {

}

Device::~Device() {
    close();
}

void Device::hardwareReset() {
    close();

    try {
        std::scoped_lock lock(device_mutex_);
        device_.hardware_reset();
    }
    catch(const std::exception& ex) {
        error("An exception has been thrown trying reset device <{}>: {}", serial_no_, ex.what());
    }
}

std::string Device::getSerialNo() const {
    return serial_no_;
}

bool Device::configureStreams(const std::vector<StreamConfig>& configs, bool exact_fps) {
    std::scoped_lock lock(device_mutex_);
    try {
        std::unordered_map<std::string, std::vector<StreamDefinition>> sensor_streams;

        for (const auto& config: configs) {

            auto stream = configureStream(config, exact_fps);
            if (!stream) {
                error("Failed to configure stream.");
                return false;
            }

            std::string sensor_name = stream->sensor.get_info(RS2_CAMERA_INFO_NAME);
            sensor_streams[sensor_name].push_back(*stream);
        }

        stream_num_ = configs.size();
        sensor_streams_ = sensor_streams;

        return true;
    }
    catch(const std::exception& ex) {
        error("An exception has been thrown trying to config streams: {}", ex.what());
    }
    catch(...) {
        error("Unknown exception has occured trying to config streams!");
    }

    return false;
}

std::vector<rs2::sensor> Device::getConfiguredSensors() {
    std::vector<rs2::sensor> sensors;
    for (auto& sensor: sensor_streams_) {
        if (sensor.second.empty()) {
            continue;
        }

        sensors.push_back(sensor.second.front().sensor);
    }
    return sensors;
}

std::optional<rs2::stream_profile> Device::getProfile(const StreamIndex& stream) {
    for (const auto& sensor: sensors_) {
        const auto& profiles = sensor.get_stream_profiles();
        for (const auto& profile: profiles) {
            if (profile.stream_type() == stream.stream && profile.stream_index() == stream.index) {
                return profile;
            }
        }
    }

    return {};
}

bool Device::open() {
    std::scoped_lock lock(device_mutex_);
    if (is_open_) {
        return true;
    }
    try {

        // the streams for a given sensor module need to be opened together
        for (auto& sensor: sensor_streams_) {
            if (sensor.second.empty()) {
                continue;
            }

            std::vector<rs2::stream_profile> profiles;
            for (const auto& stream: sensor.second) {
                profiles.push_back(stream.profile);
            }

            info("Opening streams on {} sensor", sensor.first);
            sensor.second.front().sensor.open(profiles);
        }

        is_open_ = true;
        return true;
    }
    catch(const std::exception& ex) {
        error("An exception has been thrown trying to open streams: {}", ex.what());
    }
    catch(...) {
        error("Unknown exception has occured trying to open streams!");
    }

    return false;
}

bool Device::close() {
    std::scoped_lock lock(device_mutex_);
    if (!is_open_) {
        return true;
    }
    try {

        // close the streams on each sensor module
        for (auto& sensor: sensor_streams_) {
            if (sensor.second.empty()) {
                continue;
            }

            if (is_streaming_) {
                info("Stopping streams on {} sensor ...", sensor.first);
                sensor.second.front().sensor.stop();
            }

            info("Closing {} sensor ...", sensor.first);
            sensor.second.front().sensor.close();
        }

        is_streaming_ = false;
        is_open_ = false;
        return true;
    }
    catch(const std::exception& ex) {
        error("An exception has been thrown trying to close streams: {}", ex.what());
    }
    catch(...) {
        error("Unknown exception has occured trying to close streams!");
    }

    return false;
}

bool Device::start() {
    std::scoped_lock lock(device_mutex_);
    if (!is_open_) {
        return false;
    }
    if (is_streaming_) {
        return true;
    }
    try {

        // start the streams on each sensor module
        for (auto& sensor: sensor_streams_) {
            if (sensor.second.empty()) {
                continue;
            }

            frameset_ = std::make_shared<Frameset>();

            debug("Starting streams on {} sensor ...", sensor.first);
            sensor.second.front().sensor.start([this](rs2::frame frame){ frameCallback(frame); });
        }

        is_streaming_ = true;
        return true;
    }
    catch(const std::exception& ex) {
        error("An exception has been thrown trying to config stream: {}", ex.what());
    }
    catch(...) {
        error("Unknown exception has occured trying to config stream!");
    }

    return false;
}

bool Device::stop() {
    std::scoped_lock lock(device_mutex_);
    if (!is_streaming_) {
        return true;
    }
    try {

        // stop the streams on each sensor module
        for (auto& sensor: sensor_streams_) {
            if (sensor.second.empty()) {
                continue;
            }

            debug("Stopping streams on {} sensor ...", sensor.first);
            sensor.second.front().sensor.stop();
        }

        is_streaming_ = false;
        return true;
    }
    catch(const std::exception& ex) {
        error("An exception has been thrown trying to stop streams: {}", ex.what());
    }
    catch(...) {
        error("Unknown exception has occured trying to stop streams!");
    }

    return false;
}

void Device::setCallback(FrameCallback callback) {
    frame_callback_ = callback;
}

Frameset::Ptr Device::getFrames(int timeout_ms) {
    if (is_streaming_) {
        warn("Unable to get frames, device is already streaming");
        return {};
    }

    // enable polling mode
    {
        std::scoped_lock lock(frameset_mutex_);
        waiting_for_frames_ = true;
    }

    // open the device streams if they are not already open
    // NOTE: there is a delay between opening the streams and starting to
    //       recieve data
    bool is_open = is_open_;
    if (!is_open) {
        open();
    }

    start();

    // wait up to the specified timeout for notification that the frameset has
    // been populated
    Frameset::Ptr frames;
    auto now = std::chrono::system_clock::now();
    auto timeout = now + std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lock(frameset_mutex_);
    if(frameset_condition_.wait_until(lock, timeout, [this](){
        return frameset_ && frameset_->size() >= stream_num_;
    })) {
        // copy a reference to the class level frameset
        frames = frameset_;
    }
    else {
        warn("Timed out waiting for frame data.  Timeout: {}ms", timeout_ms);
    }

    // clear the class level frameset
    frameset_ = std::make_shared<Frameset>();
    lock.unlock();

    stop();

    // close the device streams if they were closed before the call to getFrames
    if (!is_open) {
        close();
    }

    // disable polling mode
    {
        std::scoped_lock lock(frameset_mutex_);
        waiting_for_frames_ = false;
    }

    return frames;
}

Device::Ptr Device::find(const std::string& serial_no) {
    auto device = std::shared_ptr<Device>(new Device(serial_no));
    if (device->connect()) {
        return device;
    }

    return {};
}

bool Device::connect() {
    auto devices = context_.query_devices();

    debug("Found {} device(s)", devices.size());

    bool found = false;
    for (size_t i = 0; i < devices.size(); i++) {
        rs2::device device;
        try {
            device = devices[i];
        }
        catch(const std::exception& e) {
            warn("Device {} of {} failed with exception: {} ", i + 1, devices.size(), e.what());
            continue;
        }

        auto serial_no = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        debug("  Checking device: <{}>", serial_no);

        if (serial_no_.empty() || serial_no_ == serial_no) {
            device_ = device;
            found = true;
            serial_no_ = serial_no;
            info("Found matching device: <{}>", serial_no);
            break;
        }
    }

    if (found) {
        try {
            firmware_version_ = device_.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
            model_ = device_.get_info(RS2_CAMERA_INFO_NAME);
            physical_port_ = device_.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
            product_id_ = device_.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
            serial_no_ == device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

            info("Firmware version: {}", firmware_version_);
            info("Model: {}", model_);
            info("Physical port: {}", physical_port_);
            info("Product id: {}", product_id_);

            sensors_ = device_.query_sensors();
            info("Found {} sensor modules", sensors_.size());
            for(const auto& sensor : sensors_) {
                auto sensor_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
                auto profiles = sensor.get_stream_profiles();
                info("  <{}> with {} profiles", sensor_name, profiles.size());
                for (const auto& profile : profiles) {
                    auto vp = profile.as<rs2::video_stream_profile>();
                    trace("    {} {} {}x{} {}fps", vp.stream_name(), rs2_format_to_string(vp.format()), vp.width(),
                        vp.height(), vp.fps());
                }
            }

            return true;
        }
        catch(const std::exception& ex) {
            error("An exception has been thrown trying to access device <{}>: {}", serial_no_, ex.what());
        }
        catch(...) {
            error("Unknown exception has occured trying to access device <{}>!", serial_no_);
        }
    }

    return false;
}

std::optional<StreamDefinition> Device::configureStream(const StreamConfig& config, bool exact_fps) {
    info("Configuring stream <{}>: width={} height={} fps={} format={}", toString(config.stream),
        config.width, config.height, config.fps, rs2_format_to_string(config.format));

    std::optional<StreamDefinition> best_match;
    int best_fps_offset = 1024;
    auto sensors = device_.query_sensors();
    for(const auto& sensor : sensors) {
        auto profiles = sensor.get_stream_profiles();
        for (const auto& profile : profiles) {
            auto vp = profile.as<rs2::video_stream_profile>();
            if (vp.stream_type() == config.stream.stream &&
                vp.stream_index() == config.stream.index &&
                vp.width() == config.width &&
                vp.height() == config.height &&
                vp.format() == config.format)
            {
                StreamDefinition stream_def;
                stream_def.stream = config.stream;
                stream_def.sensor = sensor;
                stream_def.profile = vp;

                int fps_offset = std::abs(config.fps - vp.fps());
                if (fps_offset == 0) {
                    return stream_def;
                }
                else if (!best_match || fps_offset < best_fps_offset) {
                    best_fps_offset = fps_offset;
                    best_match = stream_def;
                }
            }
        }
    }

    return best_match;
}

void Device::frameCallback(rs2::frame frame) {
    std::scoped_lock frame_lock(frameset_mutex_);

    if (!frameset_) {
        frameset_ = std::make_shared<Frameset>();
    }

    auto profile = frame.get_profile();
    trace("Received frame (type={}:{} stamp={}) {} of {}",
        profile.stream_type(), profile.stream_index(), frame.get_timestamp(), frameset_->size(), stream_num_);

    frameset_->addFrame(frame);

    // check if the frameset is complete
    if (frameset_->size() >= stream_num_) {
        if (waiting_for_frames_) {
            // notify condition
            frameset_condition_.notify_all();
        }
        else {
            // call aggregated callback
            if (frame_callback_) {
                frame_callback_(frameset_);
            }
            frameset_ = std::make_shared<Frameset>();
        }
    }
}

}  // namespace rstream
