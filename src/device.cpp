#include <rstream/device.h>

#include <rstream/util.h>
#include <spdlog/spdlog.h>

using namespace spdlog;

namespace rstream {

Device::Device(const std::string& serial_no) : serial_no_(serial_no) {

}

Device::~Device() {
    closeStreams();
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

            auto sensors = device_.query_sensors();
            info("Found {} sensor modules", sensors.size());
            for(const auto& sensor : sensors) {
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
            error("An exception has been thrown trying to access device <{}>: ", serial_no_, ex.what());
        }
        catch(...) {
            error("Unknown exception has occured trying to access device <{}>!", serial_no_);
        }
    }

    return false;
}

Device::Ptr Device::find(const std::string& serial_no) {
    auto device = std::shared_ptr<Device>(new Device(serial_no));
    if (device->connect()) {
        return device;
    }

    return {};
}

bool Device::startStreams() {
    if (is_streaming_) {
        return true;
    }
    try {
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

bool Device::stopStreams() {
    if (!is_streaming_) {
        return true;
    }
    try {
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

bool Device::configureStreams(const std::vector<StreamConfig>& configs) {
    try {
        std::unordered_map<std::string, std::vector<StreamDefinition>> sensor_streams;

        for (const auto& config: configs) {

            auto stream = configureStream(config);
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

bool Device::openStreams() {
    if (is_open_) {
        return true;
    }
    try {
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

Frameset::Ptr Device::getFrames(int timeout_ms) {
    if (is_streaming_) {
        warn("Unable to get frames, device is already streaming");
        return {};
    }

    {
        std::scoped_lock lock(frameset_mutex_);
        waiting_for_frames_ = true;
    }

    bool is_open = is_open_;

    if (!is_open) {
        openStreams();
    }
    startStreams();

    Frameset::Ptr frames;
    auto now = std::chrono::system_clock::now();
    auto timeout = now + std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lock(frameset_mutex_);
    if(frameset_condition_.wait_until(lock, timeout, [this](){
        return frameset_ && frameset_->size() >= stream_num_;
    })) {
        frames = frameset_;
    }
    else {
        warn("Timedout waiting for frame data");
    }
    frameset_ = std::make_shared<Frameset>();
    lock.unlock();

    stopStreams();
    if (!is_open) {
        closeStreams();
    }

    {
        std::scoped_lock lock(frameset_mutex_);
        waiting_for_frames_ = false;
    }

    return frames;
}

std::optional<StreamDefinition> Device::configureStream(const StreamConfig& config) {
    info("Configuring stream <{}>: width={} height={} fps={} format={}", toString(config.stream, config.index),
        config.width, config.height, config.fps, rs2_format_to_string(config.format));
    auto sensors = device_.query_sensors();
    for(const auto& sensor : sensors) {
        auto profiles = sensor.get_stream_profiles();
        for (const auto& profile : profiles) {
            auto vp = profile.as<rs2::video_stream_profile>();
            if (vp.stream_type() == config.stream &&
                vp.stream_index() == config.index &&
                vp.width() == config.width &&
                vp.height() == config.height &&
                vp.fps() == config.fps &&
                vp.format() == config.format)
            {
                StreamDefinition stream_def;
                stream_def.stream = config.stream;
                stream_def.index = config.index;
                stream_def.sensor = sensor;
                stream_def.profile = vp;
                return stream_def;
            }
        }
    }

    return {};
}

bool Device::closeStreams() {
    if (!is_open_) {
        return true;
    }
    try {
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

void Device::frameCallback(rs2::frame frame) {
    std::scoped_lock lock(frameset_mutex_);

    if (!frameset_) {
        frameset_ = std::make_shared<Frameset>();
    }

    auto profile = frame.get_profile();
    trace("Received frame (type={}:{} stamp={}) {} of {}",
        profile.stream_type(), profile.stream_index(), frame.get_timestamp(), frameset_->size(), stream_num_);

    frameset_->addFrame(frame, profile.stream_type(), profile.stream_index());

    if (frameset_->size() >= stream_num_) {
        if (waiting_for_frames_) {
            // notify condition
            frameset_condition_.notify_all();
        }
        else {
            // TODO aggregated callback
            frameset_ = std::make_shared<Frameset>();
        }
    }
}

}  // namespace rstream
