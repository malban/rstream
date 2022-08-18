find_package(spdlog)
if (${spdlog_FOUND})
    add_custom_target(spdlog ALL)
else() 
    include(ExternalProject)
    ExternalProject_Add(spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.10.0
        GIT_SHALLOW TRUE
        GIT_CONFIG advice.detachedHead=false
        BUILD_COMMAND cmake -E echo "Skipping build step."
        INSTALL_COMMAND cmake -E echo "Skipping install step."
    )

    ExternalProject_Get_property(spdlog SOURCE_DIR)

    set(spdlog_INCLUDE_DIRS ${SOURCE_DIR}/include)
endif(${spdlog_FOUND})

