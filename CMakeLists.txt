cmake_minimum_required(VERSION 3.18)
project(eir_loco_runtime VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET EIR::lowlevel_sdk)
    set(EIR_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../sdk" CACHE PATH "Prebuilt SDK directory")
    option(EIR_WITH_HARDWARE "Build hardware support (not hardware-validated)" OFF)
    option(EIR_WITH_MUJOCO "Build MuJoCo support" ON)
    include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/EirSdk.cmake")
    include(CTest)
endif()
option(EIR_RUNTIME_WITH_ROS2 "Build the optional read-only ROS 2 publisher" OFF)
option(EIR_RUNTIME_BUILD_EXAMPLES "Build the simple simulation example" ON)
find_package(Threads REQUIRED)
add_library(eir_loco_runtime SHARED src/runtime.cpp src/backends.cpp src/flight_recorder.cpp)
add_library(eir::loco_runtime ALIAS eir_loco_runtime)
target_compile_features(eir_loco_runtime PUBLIC cxx_std_17)
target_include_directories(eir_loco_runtime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(eir_loco_runtime PUBLIC EIR::lowlevel_sdk PRIVATE Threads::Threads)
execute_process(COMMAND git rev-parse --short HEAD WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE EIR_RUNTIME_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
execute_process(COMMAND git diff --quiet WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" RESULT_VARIABLE EIR_RUNTIME_DIRTY ERROR_QUIET)
if(EIR_RUNTIME_DIRTY EQUAL 1)
    string(APPEND EIR_RUNTIME_REVISION "-worktree")
endif()
if(NOT EIR_RUNTIME_REVISION)
    set(EIR_RUNTIME_REVISION unknown)
endif()
target_compile_definitions(eir_loco_runtime PRIVATE
    EIR_RUNTIME_REVISION="${EIR_RUNTIME_REVISION}"
    EIR_HARDWARE_EQR=$<STREQUAL:${EIR_HARDWARE_FAMILY},eqr11>
    EIR_WITH_HARDWARE=$<BOOL:${EIR_WITH_HARDWARE}>
    EIR_WITH_MUJOCO=$<BOOL:${EIR_WITH_MUJOCO}>)
target_compile_options(eir_loco_runtime PRIVATE -Wall -Wextra -Wpedantic)
if(EIR_WITH_HARDWARE)
    find_package(yaml-cpp REQUIRED)
    target_link_libraries(eir_loco_runtime PRIVATE EIR::base_sdk yaml-cpp)
endif()
if(EIR_WITH_MUJOCO)
    target_link_libraries(eir_loco_runtime PRIVATE EIR::mujoco_sdk)
endif()
set_target_properties(eir_loco_runtime PROPERTIES
    EXPORT_NAME loco_runtime
    BUILD_RPATH "${EIR_SDK_ROOT}/lib"
    INSTALL_RPATH "$ORIGIN/../sdk/lib"
    INSTALL_RPATH_USE_LINK_PATH FALSE)

if(EIR_WITH_HARDWARE AND EIR_HARDWARE_FAMILY STREQUAL "eqr11")
    set_property(TARGET eir_loco_runtime APPEND PROPERTY INSTALL_RPATH
        "$ORIGIN/../sdk/variants/eqr11/${CMAKE_SYSTEM_PROCESSOR}/1.1.0/lib")
endif()

if(EIR_RUNTIME_WITH_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(diagnostic_msgs REQUIRED)
    add_library(eir_loco_runtime_ros2 SHARED ros2/state_publisher.cpp)
    add_library(eir::loco_runtime_ros2 ALIAS eir_loco_runtime_ros2)
    target_include_directories(eir_loco_runtime_ros2 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ros2/include>
        $<INSTALL_INTERFACE:include>)
    target_link_libraries(eir_loco_runtime_ros2 PUBLIC eir_loco_runtime)
    ament_target_dependencies(eir_loco_runtime_ros2 PUBLIC rclcpp sensor_msgs diagnostic_msgs)
    set_target_properties(eir_loco_runtime_ros2 PROPERTIES
        EXPORT_NAME loco_runtime_ros2 INSTALL_RPATH "$ORIGIN")
endif()

if(EIR_RUNTIME_BUILD_EXAMPLES AND EIR_WITH_MUJOCO)
    add_executable(eir_runtime_example examples/runtime_example.cpp)
    target_link_libraries(eir_runtime_example PRIVATE eir_loco_runtime Threads::Threads)
endif()
if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(NAME flight_decoder_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_flight_decoder.py)
    endif()
    add_executable(flight_recorder_benchmark tests/flight_recorder_benchmark.cpp)
    target_link_libraries(flight_recorder_benchmark PRIVATE eir_loco_runtime Threads::Threads)
    add_executable(flight_recorder_test tests/flight_recorder_test.cpp src/flight_recorder.cpp)
    target_compile_features(flight_recorder_test PRIVATE cxx_std_17)
    target_include_directories(flight_recorder_test PRIVATE include)
    target_compile_definitions(flight_recorder_test PRIVATE EIR_FLIGHT_TEST_HOOKS=1)
    target_link_libraries(flight_recorder_test PRIVATE GTest::gtest_main Threads::Threads)
    add_test(NAME flight_recorder_test COMMAND flight_recorder_test)
    add_executable(runtime_test tests/runtime_test.cpp)
    target_compile_definitions(runtime_test PRIVATE EIR_HARDWARE_EQR=$<STREQUAL:${EIR_HARDWARE_FAMILY},eqr11>
    EIR_WITH_HARDWARE=$<BOOL:${EIR_WITH_HARDWARE}>)
    target_link_libraries(runtime_test PRIVATE eir_loco_runtime GTest::gtest_main Threads::Threads)
    add_test(NAME runtime_test COMMAND runtime_test)
    if(EIR_WITH_HARDWARE AND EIR_HARDWARE_FAMILY STREQUAL "eqr11")
        add_library(eqr_abi_mismatch SHARED tests/eqr_abi_mismatch.cpp)
        target_link_libraries(eqr_abi_mismatch PRIVATE EIR::base_sdk)
        add_test(NAME eqr_loaded_abi_mismatch COMMAND runtime_test
            --gtest_filter=RuntimeEqr.RealBackendRejectsInvalidProfileBeforeOpeningTransport)
        set_tests_properties(eqr_loaded_abi_mismatch PROPERTIES
            ENVIRONMENT "LD_PRELOAD=$<TARGET_FILE:eqr_abi_mismatch>;EIR_TEST_EQR_ABI_MISMATCH=1")
    endif()
    if(EIR_WITH_MUJOCO)
        add_executable(runtime_simulation_test tests/simulation_test.cpp)
        target_link_libraries(runtime_simulation_test PRIVATE eir_loco_runtime GTest::gtest_main Threads::Threads)
        target_compile_definitions(runtime_simulation_test PRIVATE
            TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/one_joint.xml"
            SUSPENDED_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/suspended_base.xml")
        add_test(NAME runtime_simulation_test COMMAND runtime_simulation_test)
    endif()
    if(EIR_RUNTIME_WITH_ROS2)
        add_executable(runtime_ros2_test tests/ros2_test.cpp)
        target_link_libraries(runtime_ros2_test PRIVATE eir_loco_runtime_ros2 GTest::gtest_main)
        add_test(NAME runtime_ros2_test COMMAND runtime_ros2_test)
    endif()
endif()

include(CMakePackageConfigHelpers)
install(TARGETS eir_loco_runtime EXPORT EirLocoRuntimeTargets LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
if(EIR_RUNTIME_WITH_ROS2)
    install(TARGETS eir_loco_runtime_ros2 EXPORT EirLocoRuntimeTargets LIBRARY DESTINATION lib)
    install(DIRECTORY ros2/include/ DESTINATION include)
endif()
install(EXPORT EirLocoRuntimeTargets NAMESPACE eir:: DESTINATION lib/cmake/eir_loco_runtime)
configure_package_config_file(cmake/eir_loco_runtimeConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/eir_loco_runtimeConfig.cmake"
    INSTALL_DESTINATION lib/cmake/eir_loco_runtime)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/eir_loco_runtimeConfigVersion.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/eir_loco_runtimeConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/eir_loco_runtimeConfigVersion.cmake"
    DESTINATION lib/cmake/eir_loco_runtime)

install(PROGRAMS tools/解码飞行记录.py DESTINATION bin RENAME eir-decode-flight-record)
