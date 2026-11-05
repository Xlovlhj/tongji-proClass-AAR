cmake_minimum_required(VERSION 3.16)
project(openloong_isaac_controller LANGUAGES CXX)

get_filename_component(_default_root "${CMAKE_CURRENT_LIST_DIR}/../../OpenLoong-Dyn-Control" ABSOLUTE)
set(OPENLOONG_ROOT "${_default_root}" CACHE PATH "OpenLoong-Dyn-Control source checkout")
if(NOT EXISTS "${OPENLOONG_ROOT}/algorithm/mpc.cpp")
    message(FATAL_ERROR "OPENLOONG_ROOT does not contain algorithm/mpc.cpp")
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(OPENLOONG_MPC_HORIZON_STEPS 10 CACHE STRING "MPC prediction horizon in discrete steps")
if(NOT OPENLOONG_MPC_HORIZON_STEPS MATCHES "^[0-9]+$")
    message(FATAL_ERROR "OPENLOONG_MPC_HORIZON_STEPS must be an integer")
endif()
if(OPENLOONG_MPC_HORIZON_STEPS LESS 3 OR OPENLOONG_MPC_HORIZON_STEPS GREATER 30)
    message(FATAL_ERROR "OPENLOONG_MPC_HORIZON_STEPS must be in [3, 30]")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(_arch lin_arm64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
    set(_arch lin_x64)
else()
    message(FATAL_ERROR "No bundled native libraries for ${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(_third "${OPENLOONG_ROOT}/third_party")

# Keep the upstream checkout unchanged. These two source-level portability fixes
# are generated into the build directory and guarded against upstream changes.
# Pin_KinDyn originally opens a JSON relative to process cwd. Do not change cwd
# inside a library loaded by Isaac, which is multithreaded.
file(READ "${OPENLOONG_ROOT}/algorithm/pino_kin_dyn.cpp" _pino)
set(_old_json "std::ifstream in(\"joint_ctrl_config.json\", std::ios::binary);")
string(FIND "${_pino}" "${_old_json}" _json_at)
if(_json_at EQUAL -1)
    message(FATAL_ERROR "Upstream Pin_KinDyn JSON loader changed; review native path adaptation")
endif()
string(REPLACE "${_old_json}"
    "std::ifstream in((std::filesystem::path(urdf_pathIn).parent_path().parent_path() / \"common/joint_ctrl_config.json\").string(), std::ios::binary);"
    _pino "${_pino}")
# GENERATE preserves timestamps when content is unchanged, keeping repeated
# launcher/configure invocations from recompiling these template-heavy units.
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/pino_kin_dyn.cpp"
    CONTENT "#include <filesystem>\n${_pino}")

# The original predictor hardcodes 0.4 seconds, even if GaitScheduler differs.
file(READ "${OPENLOONG_ROOT}/algorithm/mpc.cpp" _mpc)
set(_old_phase "aa = i * dt / 0.4;")
string(FIND "${_mpc}" "${_old_phase}" _phase_at)
if(_phase_at EQUAL -1)
    message(FATAL_ERROR "Upstream MPC phase prediction changed; review swing-time adaptation")
endif()
string(REPLACE "${_old_phase}" "aa = i * dt / Data.tSwing;" _mpc "${_mpc}")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/mpc.cpp" CONTENT "${_mpc}")

add_library(openloong_controller SHARED
    controller_bridge.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/pino_kin_dyn.cpp"
    "${CMAKE_CURRENT_BINARY_DIR}/mpc.cpp"
    "${OPENLOONG_ROOT}/algorithm/wbc_priority.cpp"
    "${OPENLOONG_ROOT}/algorithm/priority_tasks.cpp"
    "${OPENLOONG_ROOT}/algorithm/gait_scheduler.cpp"
    "${OPENLOONG_ROOT}/algorithm/foot_placement.cpp"
    "${OPENLOONG_ROOT}/algorithm/joystick_interpreter.cpp"
    "${OPENLOONG_ROOT}/algorithm/StateEst.cpp"
    "${OPENLOONG_ROOT}/algorithm/Eul_W_filter.cpp"
    "${OPENLOONG_ROOT}/common/PVT_ctrl.cpp"
    "${OPENLOONG_ROOT}/math/useful_math.cpp"
    "${OPENLOONG_ROOT}/math/LPF_fst.cpp"
    "${OPENLOONG_ROOT}/math/bezier_1D.cpp"
    "${OPENLOONG_ROOT}/math/ramp_trajectory.cpp")
target_include_directories(openloong_controller PRIVATE
    "${OPENLOONG_ROOT}/algorithm" "${OPENLOONG_ROOT}/common" "${OPENLOONG_ROOT}/math"
    "${_third}/eigen3" "${_third}/pinocchio" "${_third}/boost"
    "${_third}/jsoncpp" "${_third}/qpOASES")
# Fixed-size Eigen fields in upstream modules otherwise contain uninitialized
# values before their first sensor update; define consistently for every TU.
target_compile_definitions(openloong_controller PRIVATE
    EIGEN_INITIALIZE_MATRICES_BY_ZERO
    EIGEN_STACK_ALLOCATION_LIMIT=0
    OPENLOONG_MPC_N=${OPENLOONG_MPC_HORIZON_STEPS})
target_compile_options(openloong_controller PRIVATE -Wall -Wextra -Wno-sign-compare -Wno-unused-parameter)
target_link_libraries(openloong_controller PRIVATE
    "${_third}/pinocchio/libpinocchio_${_arch}.a"
    "${_third}/urdfdom/liburdfdom_model_${_arch}.a"
    "${_third}/urdfdom/libtinyxml_${_arch}.a"
    "${_third}/urdfdom/libconsole_bridge_${_arch}.a"
    "${_third}/jsoncpp/libjsoncpp_${_arch}.a"
    "${_third}/qpOASES/${_arch}/libqpOASES.so.3.2"
    pthread)
target_link_options(openloong_controller PRIVATE -Wl,--no-undefined
    "-Wl,--version-script=${CMAKE_CURRENT_LIST_DIR}/exports.map")
set_property(TARGET openloong_controller APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/exports.map")
set_target_properties(openloong_controller PROPERTIES
    BUILD_RPATH "${_third}/qpOASES/${_arch}"
    INSTALL_RPATH "${_third}/qpOASES/${_arch}")
