cmake_minimum_required(VERSION 3.16)
project(pickupwinderd LANGUAGES CXX C)

# ---------------------------------------------------------------------------
# Toolchain selection
#   Default: native (development / CI host build)
#   Cross:   arm-linux-gnueabihf  (BBB target)
#     cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-linux-gnueabihf.cmake ..
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Compile options
# ---------------------------------------------------------------------------
add_compile_options(
    -Wall
    -Wextra
    -Wno-unused-parameter
    -fno-exceptions
)

# ---------------------------------------------------------------------------
# Include paths
# ---------------------------------------------------------------------------
set(BBB_LINUX_INCLUDE  ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(BBB_PRU_INCLUDE    ${CMAKE_CURRENT_SOURCE_DIR}/../pru/include)
set(APP_INCLUDE        ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
set(APP_SRC            ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# ---------------------------------------------------------------------------
# HAL library: IpcChannel + StepperPRU + LateralPRU
# ---------------------------------------------------------------------------
add_library(bbb_hal STATIC
    src/IpcChannel.cpp
    src/MoveQueue.cpp
    src/StepperPRU.cpp
    src/LateralPRU.cpp
)
target_include_directories(bbb_hal PUBLIC
    ${BBB_LINUX_INCLUDE}
    ${BBB_PRU_INCLUDE}
    ${APP_INCLUDE}
)

# ---------------------------------------------------------------------------
# Application sources (shared with all ports)
# ---------------------------------------------------------------------------
set(APP_SOURCES
    ${APP_SRC}/CommandController.cpp
    ${APP_SRC}/CommandRegistry.cpp
    ${APP_SRC}/Diag.cpp
    ${APP_SRC}/SessionController.cpp
    ${APP_SRC}/WebInterface.cpp
    ${APP_SRC}/WinderApp.Commands.cpp
    ${APP_SRC}/WinderApp.Core.cpp
    ${APP_SRC}/WinderApp.GeometryPattern.cpp
    ${APP_SRC}/WinderApp.Recipe.cpp
    ${APP_SRC}/WinderApp.Telemetry.cpp
    ${APP_SRC}/WindingPattern.cpp
    ${APP_SRC}/WindingRecipeStore.cpp
)

# ---------------------------------------------------------------------------
# Daemon executable
# ---------------------------------------------------------------------------
add_executable(pickupwinderd
    ${APP_SOURCES}
    src/main_bbb.cpp          # BBB-specific entry point (NOT src/main.cpp)
)
target_include_directories(pickupwinderd PRIVATE
    ${BBB_LINUX_INCLUDE}
    ${BBB_PRU_INCLUDE}
    ${APP_INCLUDE}
)
target_link_libraries(pickupwinderd
    PRIVATE
        bbb_hal
        pthread
)

# ---------------------------------------------------------------------------
# Optional: ArduinoJson (header-only, expected in /usr/local/include or vcpkg)
# ---------------------------------------------------------------------------
find_path(ARDUINOJSON_INCLUDE ArduinoJson.h
    HINTS /usr/local/include /usr/include
    PATH_SUFFIXES ArduinoJson
    NO_DEFAULT_PATH
)
if(ARDUINOJSON_INCLUDE)
    target_include_directories(pickupwinderd PRIVATE ${ARDUINOJSON_INCLUDE})
    target_include_directories(bbb_hal       PRIVATE ${ARDUINOJSON_INCLUDE})
    target_compile_definitions(pickupwinderd PRIVATE HAS_ARDUINOJSON=1)
endif()

# ---------------------------------------------------------------------------
# Unit tests (host-only, with gtest)
# ---------------------------------------------------------------------------
option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    find_package(GTest QUIET)
    if(GTest_FOUND OR TARGET GTest::gtest)
        add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../test test_build)
    else()
        message(STATUS "GTest not found – skipping unit tests")
    endif()
endif()

# ---------------------------------------------------------------------------
# Install (on BBB)
# ---------------------------------------------------------------------------
install(TARGETS pickupwinderd RUNTIME DESTINATION /usr/local/bin)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../data/
        DESTINATION /var/lib/pickupwinder/www)
