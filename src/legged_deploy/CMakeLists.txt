cmake_minimum_required(VERSION 3.16)
project(legged_deploy_go2 VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(USE_CMAKE "Build without ROS" ON)
option(USE_MUJOCO "Build Go2 and BPX MuJoCo executable" ON)
option(BUILD_GO2_REAL "Build Go2 real executable" ON)
option(BUILD_BPX_REAL "Build BPX real executable" OFF)
option(USE_TORCH "Enable the TorchScript inference backend" ON)
option(USE_ONNX "Enable the ONNX Runtime CPU inference backend" ON)

if(NOT USE_CMAKE)
    message(FATAL_ERROR "This deployment uses USE_CMAKE=ON and does not configure ROS.")
endif()
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "This deployment build targets Linux.")
endif()
if(NOT USE_MUJOCO AND NOT BUILD_GO2_REAL AND NOT BUILD_BPX_REAL)
    message(FATAL_ERROR "Enable at least one deployment executable: USE_MUJOCO, BUILD_GO2_REAL or BUILD_BPX_REAL.")
endif()
if(NOT USE_TORCH AND NOT USE_ONNX)
    message(FATAL_ERROR "Enable at least one inference backend: USE_TORCH or USE_ONNX.")
endif()

get_filename_component(DEPLOY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/library/core")
set(SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/unitree/unitree_sdk2")
set(LIBTORCH_DIR "${DEPLOY_ROOT}/library/inference_runtime/libtorch")
set(ONNX_RUNTIME_DIR "${DEPLOY_ROOT}/library/inference_runtime/onnxruntime")
set(MUJOCO_DIR "${DEPLOY_ROOT}/library/mujoco")
set(BPX_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/robot_sdk/bpx/bpx_sdk_open"
    CACHE PATH "BPX SDK repository containing include/ and lib/"
)
# Resolve an explicitly provided relative SDK path against this CMake source directory.
get_filename_component(BPX_SDK_DIR "${BPX_SDK_DIR}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

if(BUILD_BPX_REAL)
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" BPX_SDK_ARCH)
    if(BPX_SDK_ARCH MATCHES "^(x86_64|amd64|x64)$")
        set(BPX_SDK_ARCH x86_64)
    elseif(BPX_SDK_ARCH MATCHES "^(aarch64|arm64)$")
        set(BPX_SDK_ARCH aarch64)
    else()
        message(FATAL_ERROR "BPX SDK supports Linux x86_64 and aarch64; got ${CMAKE_SYSTEM_PROCESSOR}.")
    endif()
    set(BPX_SDK_LIBRARY "${BPX_SDK_DIR}/lib/libbpx_sdk_${BPX_SDK_ARCH}.so")
    foreach(bpx_sdk_header IN ITEMS joint_level_control.h request_robot_state.h motion_types.h)
        if(NOT EXISTS "${BPX_SDK_DIR}/include/${bpx_sdk_header}")
            message(FATAL_ERROR "BPX SDK header missing: ${BPX_SDK_DIR}/include/${bpx_sdk_header}")
        endif()
    endforeach()
    if(NOT EXISTS "${BPX_SDK_LIBRARY}")
        message(FATAL_ERROR "BPX SDK shared library missing: ${BPX_SDK_LIBRARY}")
    endif()
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

if(USE_TORCH)
    if(NOT EXISTS "${LIBTORCH_DIR}/share/cmake/Torch/TorchConfig.cmake")
        message(FATAL_ERROR "Complete matching LibTorch package missing: ${LIBTORCH_DIR}")
    endif()
    list(PREPEND CMAKE_PREFIX_PATH "${LIBTORCH_DIR}")
    find_package(Torch REQUIRED)
endif()

if(USE_ONNX)
    if(NOT EXISTS "${ONNX_RUNTIME_DIR}/include/onnxruntime_cxx_api.h" OR
       NOT EXISTS "${ONNX_RUNTIME_DIR}/include/onnxruntime_c_api.h")
        message(FATAL_ERROR "ONNX Runtime headers missing: ${ONNX_RUNTIME_DIR}/include")
    endif()
    # Restrict lookup to this project's package; do not silently use a system ORT.
    set(ONNX_RUNTIME_LIBRARY "${ONNX_RUNTIME_DIR}/lib/libonnxruntime.so")
    if(NOT EXISTS "${ONNX_RUNTIME_LIBRARY}")
        message(FATAL_ERROR "Complete matching ONNX Runtime package missing: ${ONNX_RUNTIME_LIBRARY}")
    endif()
    if(NOT EXISTS "${ONNX_RUNTIME_DIR}/lib/libonnxruntime.so.1")
        message(FATAL_ERROR "Preserve the ONNX Runtime SONAME link libonnxruntime.so.1")
    endif()
    add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
    set_target_properties(onnxruntime_prebuilt PROPERTIES
        IMPORTED_LOCATION "${ONNX_RUNTIME_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_DIR}/include"
    )
    if(EXISTS "${ONNX_RUNTIME_DIR}/VERSION_NUMBER")
        file(READ "${ONNX_RUNTIME_DIR}/VERSION_NUMBER" ONNX_RUNTIME_VERSION)
        string(STRIP "${ONNX_RUNTIME_VERSION}" ONNX_RUNTIME_VERSION)
        message(STATUS "ONNX Runtime package version: ${ONNX_RUNTIME_VERSION}")
    endif()
    message(STATUS "ONNX Runtime library: ${ONNX_RUNTIME_LIBRARY}")
endif()

find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
# Existing matplotlibcpp headers still require Python development headers.
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Python3 QUIET COMPONENTS NumPy)

if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp)
else()
    message(FATAL_ERROR "yaml-cpp CMake target was not found.")
endif()

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
set(CMAKE_INSTALL_RPATH "")
if(USE_TORCH)
    list(APPEND CMAKE_INSTALL_RPATH "${LIBTORCH_DIR}/lib")
endif()
if(USE_ONNX)
    list(APPEND CMAKE_INSTALL_RPATH "${ONNX_RUNTIME_DIR}/lib")
endif()
foreach(python_library IN LISTS Python3_LIBRARIES)
    if(IS_ABSOLUTE "${python_library}")
        get_filename_component(python_library_dir "${python_library}" DIRECTORY)
        list(APPEND CMAKE_INSTALL_RPATH "${python_library_dir}")
    endif()
endforeach()
if(BUILD_GO2_REAL)
    list(APPEND CMAKE_INSTALL_RPATH "${SDK_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")
endif()
if(USE_MUJOCO)
    list(APPEND CMAKE_INSTALL_RPATH "${MUJOCO_DIR}/lib")
endif()
if(BUILD_BPX_REAL)
    list(APPEND CMAKE_INSTALL_RPATH "${BPX_SDK_DIR}/lib")
endif()
list(REMOVE_DUPLICATES CMAKE_INSTALL_RPATH)
add_link_options("-Wl,--disable-new-dtags")

add_library(go2_core STATIC
    "${CORE_DIR}/rl_sdk/rl_sdk.cpp"
    "${CORE_DIR}/inference_runtime/inference_runtime.cpp"
    "${CORE_DIR}/observation_buffer/observation_buffer.cpp"
    "${CORE_DIR}/motion_loader/motion_loader.cpp"
)
target_include_directories(go2_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/fsm_robot"
)
foreach(component IN ITEMS rl_sdk inference_runtime observation_buffer motion_loader
        fsm loop logger vector_math matplotlibcpp)
    target_include_directories(go2_core PUBLIC "${CORE_DIR}/${component}")
endforeach()
target_compile_definitions(go2_core PUBLIC
    USE_CMAKE
    POLICY_DIR=\"${DEPLOY_ROOT}/policy\"
    CMAKE_CURRENT_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"
)
target_link_libraries(go2_core PUBLIC
    ${YAML_CPP_TARGET}
    TBB::tbb
    Threads::Threads
    Python3::Python
    Python3::Module
)
# Keep feature macros PUBLIC: the shared header conditionally contains backend members.
if(USE_TORCH)
    target_compile_definitions(go2_core PUBLIC USE_TORCH)
    target_link_libraries(go2_core PUBLIC ${TORCH_LIBRARIES})
endif()
if(USE_ONNX)
    target_compile_definitions(go2_core PUBLIC USE_ONNX)
    target_link_libraries(go2_core PUBLIC onnxruntime_prebuilt)
endif()
if(TARGET Python3::NumPy)
    target_link_libraries(go2_core PUBLIC Python3::NumPy)
else()
    target_compile_definitions(go2_core PUBLIC WITHOUT_NUMPY)
endif()

if(BUILD_GO2_REAL)
    if(NOT EXISTS "${SDK_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "Unitree SDK2 is missing at ${SDK_DIR}")
    endif()
    if(NOT EXISTS "${SDK_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_sdk2.a")
        message(FATAL_ERROR "Unitree SDK2 archive does not match ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
    set(BUILD_EXAMPLES OFF CACHE BOOL "Disable Unitree examples" FORCE)
    add_subdirectory("${SDK_DIR}" unitree_sdk2_build)
    add_executable(rl_real_go2
        src/rl_real_go2.cpp
        library/thirdparty/joystick/joystick.cc
    )
    target_include_directories(rl_real_go2 PRIVATE
        library/thirdparty/joystick
    )
    target_link_libraries(rl_real_go2 PRIVATE go2_core unitree_sdk2)
endif()

if(BUILD_BPX_REAL)
    # Import only the SDK library; do not add its example/test targets to this project.
    add_library(BPX::SDK SHARED IMPORTED GLOBAL)
    set_target_properties(BPX::SDK PROPERTIES
        IMPORTED_LOCATION "${BPX_SDK_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${BPX_SDK_DIR}/include"
        INTERFACE_LINK_LIBRARIES Threads::Threads
    )
    add_executable(rl_real_bpx
        src/rl_real_bpx.cpp
        library/thirdparty/joystick/joystick.cc
    )
    target_include_directories(rl_real_bpx PRIVATE
        library/thirdparty/joystick
    )
    # Keep the real-robot selector private: shared core/MuJoCo/Go2 must not see it.
    target_compile_definitions(rl_real_bpx PRIVATE USE_BPX_REAL)
    target_link_libraries(rl_real_bpx PRIVATE go2_core BPX::SDK)
    message(STATUS "BPX SDK architecture: ${BPX_SDK_ARCH}")
    message(STATUS "BPX SDK library: ${BPX_SDK_LIBRARY}")
endif()

if(USE_MUJOCO)
    find_package(glfw3 REQUIRED)
    if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h" OR
       NOT EXISTS "${MUJOCO_DIR}/lib/libmujoco.so.3.2.7")
        message(FATAL_ERROR "MuJoCo 3.2.7 headers/library missing: ${MUJOCO_DIR}")
    endif()
    add_library(mujoco_prebuilt SHARED IMPORTED)
    set_target_properties(mujoco_prebuilt PROPERTIES
        IMPORTED_LOCATION "${MUJOCO_DIR}/lib/libmujoco.so.3.2.7"
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include"
        IMPORTED_NO_SONAME TRUE
    )
    add_executable(rl_sim_mujoco
        src/rl_sim_mujoco.cpp
        library/thirdparty/mujoco_simulate/platform_ui_adapter.cc
        library/thirdparty/mujoco_simulate/glfw_adapter.cc
        library/thirdparty/mujoco_simulate/glfw_dispatch.cc
        library/thirdparty/mujoco_simulate/simulate.cc
        library/thirdparty/joystick/joystick.cc
    )
    target_compile_definitions(rl_sim_mujoco PRIVATE USE_MUJOCO)
    target_include_directories(rl_sim_mujoco PRIVATE
        library/thirdparty/mujoco_simulate
        library/thirdparty/joystick
    )
    target_link_libraries(rl_sim_mujoco PRIVATE
        go2_core mujoco_prebuilt glfw ${CMAKE_DL_LIBS}
    )
endif()

message(STATUS "Deployment root: ${DEPLOY_ROOT}")
message(STATUS "Inference backends: USE_TORCH=${USE_TORCH}; USE_ONNX=${USE_ONNX}")
message(STATUS "Go2 policy: ${DEPLOY_ROOT}/policy/go2/robot_lab/config.yaml (model_name selects backend)")
message(STATUS "BPX policy: ${DEPLOY_ROOT}/policy/bpx/mirrorme/config.yaml; robot settings: ${DEPLOY_ROOT}/policy/bpx/base.yaml")
message(STATUS "MuJoCo scenes: ${DEPLOY_ROOT}/src/robot_description/{go2,bpx}_description/mjcf")
message(STATUS "USE_MUJOCO=${USE_MUJOCO}; BUILD_GO2_REAL=${BUILD_GO2_REAL}; BUILD_BPX_REAL=${BUILD_BPX_REAL}")