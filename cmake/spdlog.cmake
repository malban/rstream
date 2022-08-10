include(ExternalProject)

ExternalProject_Add(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.10.0
    INSTALL_COMMAND cmake -E echo "Skipping install step."
)

ExternalProject_Get_property(spdlog SOURCE_DIR)

set(spdlog_INCLUDE_DIRS ${SOURCE_DIR}/include)
