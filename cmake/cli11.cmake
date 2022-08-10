include(ExternalProject)

ExternalProject_Add(cli11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11
    GIT_TAG v2.1.2
    INSTALL_COMMAND cmake -E echo "Skipping install step."
)

ExternalProject_Get_property(cli11 SOURCE_DIR)

set(cli11_INCLUDE_DIRS ${SOURCE_DIR}/include)
