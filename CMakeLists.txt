cmake_minimum_required(VERSION 3.10)
project(libbno055_linux VERSION 1.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SOURCES
    src/bno055.cpp
)

# Header files
set(HEADERS
    include/libbno055-linux/bno055.hpp
)

# Add library
add_library(libbno055-linux SHARED ${SOURCES} ${HEADERS})

# Prevent duplicate "lib" prefix (e.g. liblibbno055-linux.so)
set_target_properties(libbno055-linux PROPERTIES PREFIX "")

# Include directories
target_include_directories(libbno055-linux
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(libbno055-linux PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Clang-Tidy static analysis integration
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" ON)
if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
    if(CLANG_TIDY_EXE)
        set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
        message(STATUS "clang-tidy enabled: ${CLANG_TIDY_EXE}")
    else()
        message(WARNING "clang-tidy not found, static analysis disabled.")
    endif()
endif()

# Dependencies
find_package(Threads REQUIRED)
target_link_libraries(libbno055-linux PRIVATE Threads::Threads)

# Example executables
add_executable(calibrate_imu examples/calibrate.cpp)
target_link_libraries(calibrate_imu PRIVATE libbno055-linux)

add_executable(read_data_noexcept examples/read_data_noexcept.cpp)
target_link_libraries(read_data_noexcept PRIVATE libbno055-linux)

add_executable(read_all_data examples/read_all_data.cpp)
target_link_libraries(read_all_data PRIVATE libbno055-linux)

# Optional ROS 2 integration (conditional build of ROS 2 publisher samples)
find_package(rclcpp QUIET)
find_package(rclcpp_lifecycle QUIET)
find_package(sensor_msgs QUIET)
find_package(diagnostic_msgs QUIET)
find_package(std_srvs QUIET)

if(rclcpp_FOUND AND sensor_msgs_FOUND AND diagnostic_msgs_FOUND AND std_srvs_FOUND)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_pep257_FOUND TRUE)
    set(ament_cmake_xmllint_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    message(STATUS "ROS 2 environment detected. Building ROS 2 publisher node sample.")
    add_executable(bno055_publisher_node src/ros2/ros2_publisher_node.cpp)
    target_link_libraries(bno055_publisher_node PRIVATE libbno055-linux)
    target_link_libraries(bno055_publisher_node PRIVATE rclcpp::rclcpp sensor_msgs::sensor_msgs diagnostic_msgs::diagnostic_msgs std_srvs::std_srvs)

    install(TARGETS bno055_publisher_node
        DESTINATION lib/${PROJECT_NAME}
    )

    if(rclcpp_lifecycle_FOUND)
        message(STATUS "ROS 2 Lifecycle detected. Building Lifecycle Node sample.")
        # Lifecycle Node (Managed Node)
        add_executable(bno055_lifecycle_publisher_node src/ros2/ros2_lifecycle_publisher_node.cpp)
        target_link_libraries(bno055_lifecycle_publisher_node PRIVATE libbno055-linux)
        target_link_libraries(bno055_lifecycle_publisher_node PRIVATE
            rclcpp::rclcpp
            rclcpp_lifecycle::rclcpp_lifecycle
            sensor_msgs::sensor_msgs
            diagnostic_msgs::diagnostic_msgs std_srvs::std_srvs
        )

        install(TARGETS bno055_lifecycle_publisher_node
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()

    # Install ROS 2 configuration and launch files
    install(DIRECTORY config launch
        DESTINATION share/libbno055_linux
    )
else()
    message(STATUS "ROS 2 not found. Skipping ROS 2 publisher node sample.")
endif()


# Installation definitions
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Install targets
install(TARGETS libbno055-linux
    EXPORT libbno055-linuxTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS calibrate_imu read_data_noexcept read_all_data
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install headers
install(DIRECTORY include/libbno055-linux
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export targets
install(EXPORT libbno055-linuxTargets
    FILE libbno055-linuxTargets.cmake
    NAMESPACE libbno055-linux::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libbno055-linux
)

# Configure config file
configure_package_config_file(
    cmake/libbno055-linuxConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/libbno055-linuxConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libbno055-linux
)

# Write version file
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/libbno055-linuxConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Install config files
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/libbno055-linuxConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/libbno055-linuxConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libbno055-linux
)

# Testing
option(BUILD_TESTING "Build unit tests" OFF)
if(BUILD_TESTING AND NOT rclcpp_FOUND)
    # Try to find system-installed GTest (e.g., provided by apt, ROS, vcpkg, or Conan)
    # This prevents network access during offline builds (like ROS buildfarms)
    find_package(GTest QUIET)

    if(NOT GTest_FOUND)
        message(STATUS "GTest not found locally. Fetching GoogleTest from GitHub...")
        include(FetchContent)
        FetchContent_Declare(
          googletest
          GIT_REPOSITORY https://github.com/google/googletest.git
          GIT_TAG        release-1.12.1
        )
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        
        # Disable clang-tidy for third-party GoogleTest to prevent false-positive errors
        set(BACKUP_CLANG_TIDY "${CMAKE_CXX_CLANG_TIDY}")
        set(CMAKE_CXX_CLANG_TIDY "")
        FetchContent_MakeAvailable(googletest)
        set(CMAKE_CXX_CLANG_TIDY "${BACKUP_CLANG_TIDY}")
    endif()

    enable_testing()

    add_executable(libbno055-linux_tests
        tests/test_bno055.cpp
    )

    if(TARGET GTest::gtest_main)
        target_link_libraries(libbno055-linux_tests
            PRIVATE
                libbno055-linux
                GTest::gtest_main
        )
    else()
        target_link_libraries(libbno055-linux_tests
            PRIVATE
                libbno055-linux
                gtest_main
        )
    endif()

    include(GoogleTest)
    gtest_discover_tests(libbno055-linux_tests)

    # If ROS 2 is available, also build the ROS 2 node integration tests
    if(rclcpp_FOUND AND sensor_msgs_FOUND AND rclcpp_lifecycle_FOUND AND diagnostic_msgs_FOUND AND std_srvs_FOUND)
        message(STATUS "ROS 2 detected. Building ROS 2 node integration tests.")
        add_executable(libbno055-linux_ros2_tests
            tests/test_ros2_nodes.cpp
        )
        target_compile_definitions(libbno055-linux_ros2_tests PRIVATE ROS2_NODE_TESTING)
        
        if(TARGET GTest::gtest_main)
            target_link_libraries(libbno055-linux_ros2_tests
                PRIVATE
                    libbno055-linux
                    rclcpp::rclcpp
                    rclcpp_lifecycle::rclcpp_lifecycle
                    sensor_msgs::sensor_msgs
                    diagnostic_msgs::diagnostic_msgs std_srvs::std_srvs
                    GTest::gtest_main
            )
        else()
            target_link_libraries(libbno055-linux_ros2_tests
                PRIVATE
                    libbno055-linux
                    rclcpp::rclcpp
                    rclcpp_lifecycle::rclcpp_lifecycle
                    sensor_msgs::sensor_msgs
                    diagnostic_msgs::diagnostic_msgs std_srvs::std_srvs
                    gtest_main
            )
        endif()
        gtest_discover_tests(libbno055-linux_ros2_tests)
    endif()
endif()

if(rclcpp_FOUND)
    find_package(ament_cmake REQUIRED)
    ament_package()
endif()

# ---------------------------------------------------------
# CPack Packaging for non-ROS users (.deb / .rpm)
# ---------------------------------------------------------
set(CPACK_PACKAGE_NAME "libbno055-linux")
set(CPACK_PACKAGE_VENDOR "lazytatzv")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "A robust C++17 library for the BNO055 sensor over I2C.")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

# Exclude ROS components from the standalone debian package
if(rclcpp_FOUND)
    set(CPACK_COMPONENTS_ALL_IN_ONE_PACKAGE 1)
endif()

# Debian specific
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64") # Note: Will be overridden by build system if cross-compiling
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "lazytatzv")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")

set(CPACK_GENERATOR "DEB;TGZ")
include(CPack)
