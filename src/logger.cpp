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

#include <rstream/logger.h>

#include <mutex>

#define FMT_USE_STRING_VIEW
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>

namespace rstream {

template<typename Mutex>
class CustomSink : public spdlog::sinks::base_sink<Mutex>
{
    public:
    CustomSink(std::function<void(LogLevel, const std::string&)> sink) : sink_(sink) {}

    protected:
    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        if (msg.level == spdlog::level::info) {
            sink_(LogLevel::Info,  fmt::to_string(msg.payload));
        }
        else if (msg.level == spdlog::level::warn) {
            sink_(LogLevel::Warn,  fmt::to_string(msg.payload));
        }
        else if (msg.level == spdlog::level::err) {
            sink_(LogLevel::Error, fmt::to_string(msg.payload));
        }
        else {
            sink_(LogLevel::Debug, fmt::to_string(msg.payload));
        }
    }

    void flush_() override {}

    std::function<void(LogLevel, const std::string&)> sink_;
};

void sinkLogs(std::function<void(LogLevel, const std::string&)> sink) {
    if (!sink) {
        return;
    }

    spdlog::sink_ptr custom_sink = std::make_shared<CustomSink<std::mutex>>(sink);
    spdlog::default_logger()->sinks().clear();
    spdlog::default_logger()->sinks().push_back(custom_sink);
    spdlog::set_pattern("%v");
}

void setLogLevel(LogLevel level) {
    if (level == LogLevel::Info) {
        spdlog::set_level(spdlog::level::info);
    }
    else if (level == LogLevel::Warn) {
        spdlog::set_level(spdlog::level::warn);
    }
    else if (level == LogLevel::Error) {
        spdlog::set_level(spdlog::level::err);
    }
    else {
        spdlog::set_level(spdlog::level::debug);
    }
}

}  // namespace rstream
