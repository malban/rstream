include(FetchContent)
FetchContent_Declare(cli11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11
    GIT_TAG        v2.1.2
)

FetchContent_MakeAvailable(cli11)
FetchContent_GetProperties(cli11)
set(cli11_INCLUDE_DIRS ${cli11_SOURCE_DIR}/include)
