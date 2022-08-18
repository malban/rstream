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

#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <boost/algorithm/string.hpp>
#include <CLI/CLI.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rstream/device.h>
#include <rstream/util.h>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using namespace spdlog;

bool exit_ = false;

void handleSignal(int sig) {
    // push ENTER to cin in case cli is blocked by getch()
    char c = 10;
    ioctl(0, TIOCSTI, &c);
    exit_ = true;
}

char getch() {
    // turn off canonical mode and echo mode
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) return 0;
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) return 0;
    if (read(0, &buf, 1) < 0) return 0;
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) return 0;
    return buf;
}

int main(int argc, char* argv[]) {

    CLI::App app{"rstream_cli"};

    std::vector<std::string> serial_numbers;
    app.add_option("-n,--serial-no", serial_numbers, "Serial number")->take_all();

    std::vector<std::string> streams;
    app.add_option("-s,--stream", streams, "Stream configuration (TYPE:WIDTH:HEIGHT:FPS:FORMAT)")->take_all();

    std::string level = "info";
    app.add_option("-l,--level", level, "Log level")
        ->transform(CLI::IsMember({"trace", "debug", "info", "warn", "error", "off"}, CLI::ignore_case));

    std::string mode = "continuous";
    app.add_option("-m,--mode", mode, "Mode")->default_val("continuous")
        ->transform(CLI::IsMember({"continuous", "user"}, CLI::ignore_case));

    bool keep_open = false;
    app.add_flag("-k,--keep-open", keep_open, "Keep streams open");

    bool save_images = false;
    app.add_flag("--save-images", save_images, "Save images to output directory");

    std::string outdir = ".";
    app.add_option("-o,--outdir", outdir, "Output directory");

    CLI11_PARSE(app, argc, argv);

    std::vector<rstream::StreamConfig> stream_configs;
    for (const auto& stream: streams) {
        std::vector<std::string> params;
        boost::split(params, stream, boost::is_any_of(":"));
        if (params.size() != 5) {
            std::cerr << "Stream configuration must be of the format:  TYPE:WIDTH:HEIGHT:FPS:FORMAT\n\n";
            std::cerr << app.help() << std::flush;
            return 1;
        }

        rstream::StreamConfig config;

        // parse type
        auto stream_type = rstream::parseStreamType(params[0]);
        if (!stream_type) {
            std::cerr << "Stream type must be one of the following: [color, depth, infra, infra1, infra2, fisheye, confidence]\n\n";
            std::cerr << app.help() << std::flush;
            return 1;
        }
        config.stream = *stream_type;

        // parse width
        try {
            config.width = std::stoi(params[1]);
            if (config.width < 1) {
                std::cerr << "Stream width must be postitive\n\n";
                std::cerr << app.help() << std::flush;
                return 1;
            }
        }
        catch(...) {
            std::cerr << "Stream width must be an integer\n\n";
            std::cerr << app.help() << std::flush;
            return 1;
        }

        // parse height
        try {
            config.height = std::stoi(params[2]);
            if (config.height < 1) {
                std::cerr << "Stream height must be postitive\n\n";
                std::cerr << app.help() << std::flush;
                return 1;
            }
        }
        catch(...) {
            std::cerr << "Stream height must be an integer\n\n";
            std::cerr << app.help() << std::flush;
            return 1;
        }

        // parse fps
        try {
            config.fps = std::stoi(params[3]);
            if (config.fps < 1) {
                std::cerr << "Stream fps must be postitive\n\n";
                std::cerr << app.help() << std::flush;
                return 1;
            }
        }
        catch(...) {
            std::cerr << "Stream fps must be an integer\n\n";
            std::cerr << app.help() << std::flush;
            return 1;
        }

        // parse format
        auto format = rstream::parseStreamFormat(params[4]);
        if (!format) {
            std::cerr << "Stream format must be one of the following: [bgr8, bgra8, raw8, raw10, raw16, rgb8, rgba8, uyvy, y8, yuyv, z16]\n\n";
            std::cerr << app.help() << std::flush;
            return 1;
        }
        config.format = *format;

        stream_configs.push_back(config);
    }

    set_level(rstream::parseLevel(level));

    std::vector<rstream::Device::Ptr> devices;
    for (const auto& serial_no: serial_numbers) {
        auto device = rstream::Device::find(serial_no);
        if (!device) {
          error("Failed to find device with serial_no: <{}>", serial_no);
          return 1;
        }

        if (!device->configureStreams(stream_configs)) {
            error("Failed to configure streams");
            return 1;
        }

        devices.push_back(device);
    }
    if (devices.empty()) {
        auto device = rstream::Device::find("");
        if (!device) {
          error("Failed to find device");
          return 1;
        }

        devices.push_back(device);
        if (!device->configureStreams(stream_configs, false)) {
            error("Failed to configure streams");
            return 1;
        }
    }

    signal(SIGINT, handleSignal);
    signal(SIGTSTP, handleSignal);

    if (mode == "continuous") {
        for (auto& device: devices) {

            if (save_images) {
                info("setting callback for {}", device->getSerialNo());
                device->setCallback([&](rstream::Frameset::Ptr frames){
                    info("{} callback", device->getSerialNo());
                    static int num_frames = 0;
                    for (const auto& config: stream_configs) {
                        auto mat = frames->getMat(config.stream);
                        if (!mat.empty()) {
                            std::string filename = device->getSerialNo() + "_" +
                                rstream::toString(config.stream) + "_" +
                                std::to_string(num_frames) +  ".png";
                            fs::path filepath = outdir;
                            filepath /= filename;
                            cv::imwrite(filepath, mat);
                        }
                    }
                    num_frames++;
                });
            }

            if (!device->open()) {
                error("Failed to open streams");
                return 1;
            }

            if (!device->start()) {
                error("Failed to start streams");
                return 1;
            }
        }

        while (!exit_) {
            std::this_thread::sleep_for(100ms);
        }
    }
    else {

        if (keep_open) {
            for (auto& device: devices) {
                device->open();
            }
        }

        std::cout << "Press ENTER/RETURN to capture a frame.\n";

        int num_frames = 0;
        double total_elapsed = 0.0;
        while (!exit_) {
            std::this_thread::sleep_for(250ms);
            if (getch() == 10 && !exit_) {
                info("Waiting for frames ...");
                std::vector<std::thread> threads;
                auto start = std::chrono::system_clock::now();
                for (auto& device: devices) {
                    threads.emplace_back([&](){
                        auto frames = device->getFrames(1000);
                        if (!frames) {
                            warn("Failed to get frames");
                        }
                        else {
                            std::chrono::duration<double, std::ratio<1, 1000>> elapsed_ms = std::chrono::system_clock::now() - start;
                            info("Got frame in {:.2f}ms", elapsed_ms.count());

                            if (save_images) {
                                for (const auto& config: stream_configs) {
                                    auto mat = frames->getMat(config.stream);
                                    if (!mat.empty()) {
                                        std::string filename = device->getSerialNo() + "_" +
                                            rstream::toString(config.stream) + "_" +
                                            std::to_string(num_frames) +  ".png";
                                        fs::path filepath = outdir;
                                        filepath /= filename;
                                        cv::imwrite(filepath, mat);
                                    }
                                }
                            }
                        }
                    });
                }

                for (auto& thread: threads) {
                    thread.join();
                }

                std::chrono::duration<double, std::ratio<1, 1000>> elapsed_ms = std::chrono::system_clock::now() - start;
                num_frames++;
                total_elapsed += elapsed_ms.count();
                info("Got frames in {:.2f}ms ({:.2f}ms)", elapsed_ms.count(), total_elapsed / num_frames);
            }
        }
    }

    return 0;
}
