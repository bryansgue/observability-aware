cmake_minimum_required(VERSION 3.14)
project(quadrotor_nmpc_observability LANGUAGES CXX C)

# Default to Release (-O3) — solver needs it to meet 10ms budget at 100 Hz
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.2 REQUIRED)

# acados
if(DEFINED ENV{ACADOS_SOURCE_DIR})
    set(ACADOS_DIR $ENV{ACADOS_SOURCE_DIR})
else()
    set(ACADOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../acados")
endif()
message(STATUS "ACADOS_DIR = ${ACADOS_DIR}")

set(ACADOS_INCLUDE_DIR ${ACADOS_DIR}/include)
set(ACADOS_LIB_DIR     ${ACADOS_DIR}/lib)

# ROS2 (optional — only needed for SiL targets)
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(nav_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(mujoco_ros_utils QUIET)
find_package(std_msgs QUIET)
find_package(std_srvs QUIET)
find_package(geometry_msgs QUIET)
find_package(quadrotor_msgs QUIET)
find_package(mav_manager_srv QUIET)

if(rclcpp_FOUND AND quadrotor_msgs_FOUND)
    set(HAS_ROS2 TRUE)
    message(STATUS "ROS2 found — SiL targets will be built")
else()
    set(HAS_ROS2 FALSE)
    message(STATUS "ROS2 NOT found — only MiL targets will be built")
endif()

# ── Common library (quaternion, RK4, params) ─────────────────────────────────
add_library(quadrotor_common STATIC
    src/common/quaternion_algebra.cpp
    src/common/integrators.cpp
)
target_include_directories(quadrotor_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/acados
    ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${ACADOS_INCLUDE_DIR}/hpipm/include
)
target_link_libraries(quadrotor_common PUBLIC Eigen3::Eigen)

# ── Trajectory library (Lissajous, arc-length, attitude ref) ─────────────────
add_library(quadrotor_trajectory STATIC
    src/trajectory/lissajous.cpp
    src/trajectory/arc_length.cpp
    src/trajectory/attitude_reference.cpp
)
target_link_libraries(quadrotor_trajectory PUBLIC quadrotor_common)

# ── MuJoCo ROS2 interface (shared by all SiL controllers) ───────────────────
if(HAS_ROS2)
    add_library(mujoco_interface STATIC
        src/mujoco/mujoco_interface.cpp
        src/mujoco/sil_protocol.cpp
    )
    target_link_libraries(mujoco_interface PUBLIC
        quadrotor_common
    )
    ament_target_dependencies(mujoco_interface PUBLIC
        rclcpp nav_msgs sensor_msgs mujoco_ros_utils std_msgs std_srvs geometry_msgs quadrotor_msgs
    )
endif()

# ── NMPC solver (generated C code — added after running ocp_generation/) ─────
set(NMPC_CODEGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code_nmpc)
if(EXISTS ${NMPC_CODEGEN_DIR}/acados_solver_quadrotor_nmpc.c)
    message(STATUS "NMPC generated code found at ${NMPC_CODEGEN_DIR}")

    file(GLOB NMPC_MODEL_SRC   ${NMPC_CODEGEN_DIR}/quadrotor_nmpc_model/*.c)
    file(GLOB NMPC_COST_SRC    ${NMPC_CODEGEN_DIR}/quadrotor_nmpc_cost/*.c)
    file(GLOB NMPC_CONSTR_SRC  ${NMPC_CODEGEN_DIR}/quadrotor_nmpc_constraints/*.c)

    add_library(nmpc_solver STATIC
        ${NMPC_CODEGEN_DIR}/acados_solver_quadrotor_nmpc.c
        ${NMPC_MODEL_SRC}
        ${NMPC_COST_SRC}
        ${NMPC_CONSTR_SRC}
    )
    target_include_directories(nmpc_solver PUBLIC
        ${NMPC_CODEGEN_DIR}
        ${ACADOS_INCLUDE_DIR}
        ${ACADOS_INCLUDE_DIR}/acados
        ${ACADOS_INCLUDE_DIR}/blasfeo/include
        ${ACADOS_INCLUDE_DIR}/hpipm/include
    )
    target_link_libraries(nmpc_solver PUBLIC
        ${ACADOS_LIB_DIR}/libacados.so
        ${ACADOS_LIB_DIR}/libblasfeo.so
        ${ACADOS_LIB_DIR}/libhpipm.so
        m
    )

    # ── NMPC MiL simulation (standalone, no ROS2) ────────────────────
    add_executable(nmpc_sim
        src/nmpc/nmpc_sim.cpp
        src/nmpc/nmpc_controller.cpp
    )
    target_link_libraries(nmpc_sim PRIVATE
        quadrotor_common
        quadrotor_trajectory
        nmpc_solver
    )

    # ── NMPC SiL simulation (MuJoCo + ROS2) ─────────────────────────
    if(HAS_ROS2)
        add_executable(nmpc_sil
            src/nmpc/nmpc_sil.cpp
            src/nmpc/nmpc_controller.cpp
        )
        target_link_libraries(nmpc_sil
            quadrotor_common
            quadrotor_trajectory
            nmpc_solver
            mujoco_interface
        )
        ament_target_dependencies(nmpc_sil
            rclcpp nav_msgs std_msgs std_srvs geometry_msgs quadrotor_msgs
        )
    endif()

else()
    message(STATUS "NMPC generated code NOT found — run ocp_generation/generate_nmpc_ocp.py first")
endif()

# ── MPCC solver (generated C code — added after running ocp_generation/) ──────
set(MPCC_CODEGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code_mpcc)
if(EXISTS ${MPCC_CODEGEN_DIR}/acados_solver_quadrotor_mpcc.c)
    message(STATUS "MPCC generated code found at ${MPCC_CODEGEN_DIR}")

    file(GLOB MPCC_MODEL_SRC   ${MPCC_CODEGEN_DIR}/quadrotor_mpcc_model/*.c)
    file(GLOB MPCC_COST_SRC    ${MPCC_CODEGEN_DIR}/quadrotor_mpcc_cost/*.c)
    file(GLOB MPCC_CONSTR_SRC  ${MPCC_CODEGEN_DIR}/quadrotor_mpcc_constraints/*.c)

    add_library(mpcc_solver STATIC
        ${MPCC_CODEGEN_DIR}/acados_solver_quadrotor_mpcc.c
        ${MPCC_MODEL_SRC}
        ${MPCC_COST_SRC}
        ${MPCC_CONSTR_SRC}
    )
    target_include_directories(mpcc_solver PUBLIC
        ${MPCC_CODEGEN_DIR}
        ${ACADOS_INCLUDE_DIR}
        ${ACADOS_INCLUDE_DIR}/acados
        ${ACADOS_INCLUDE_DIR}/blasfeo/include
        ${ACADOS_INCLUDE_DIR}/hpipm/include
    )
    target_link_libraries(mpcc_solver PUBLIC
        ${ACADOS_LIB_DIR}/libacados.so
        ${ACADOS_LIB_DIR}/libblasfeo.so
        ${ACADOS_LIB_DIR}/libhpipm.so
        m
    )
else()
    message(STATUS "MPCC generated code NOT found — run ocp_generation/generate_mpcc_ocp.py first")
endif()

# ── MHE solver (generated C code — added after running ocp_generation/) ───────
set(MHE_CODEGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code_mhe)
if(EXISTS ${MHE_CODEGEN_DIR}/acados_solver_quadrotor_mhe.c)
    message(STATUS "MHE generated code found at ${MHE_CODEGEN_DIR}")

    file(GLOB MHE_MODEL_SRC   ${MHE_CODEGEN_DIR}/quadrotor_mhe_model/*.c)
    file(GLOB MHE_COST_SRC    ${MHE_CODEGEN_DIR}/quadrotor_mhe_cost/*.c)
    file(GLOB MHE_CONSTR_SRC  ${MHE_CODEGEN_DIR}/quadrotor_mhe_constraints/*.c)

    add_library(mhe_solver STATIC
        ${MHE_CODEGEN_DIR}/acados_solver_quadrotor_mhe.c
        ${MHE_MODEL_SRC}
        ${MHE_COST_SRC}
        ${MHE_CONSTR_SRC}
    )
    target_include_directories(mhe_solver PUBLIC
        ${MHE_CODEGEN_DIR}
        ${ACADOS_INCLUDE_DIR}
        ${ACADOS_INCLUDE_DIR}/acados
        ${ACADOS_INCLUDE_DIR}/blasfeo/include
        ${ACADOS_INCLUDE_DIR}/hpipm/include
    )
    target_link_libraries(mhe_solver PUBLIC
        ${ACADOS_LIB_DIR}/libacados.so
        ${ACADOS_LIB_DIR}/libblasfeo.so
        ${ACADOS_LIB_DIR}/libhpipm.so
        m
    )

    # ── MHE-MPCC integrated SiL (needs both solvers) ─────────────────
    if(HAS_ROS2 AND TARGET mpcc_solver)
        add_executable(mhe_mpcc_sil
            src/mhe/mhe_mpcc_sil.cpp
            src/mhe/mhe_controller.cpp
            src/mpcc/mpcc_controller.cpp
        )
        target_link_libraries(mhe_mpcc_sil
            quadrotor_common
            quadrotor_trajectory
            mhe_solver
            mpcc_solver
            mujoco_interface
            nlohmann_json::nlohmann_json
        )
        ament_target_dependencies(mhe_mpcc_sil
            rclcpp nav_msgs std_msgs std_srvs geometry_msgs quadrotor_msgs
        )
        message(STATUS "mhe_mpcc_sil target configured")
    else()
        message(STATUS "mhe_mpcc_sil: skipped (needs ROS2 + mpcc_solver)")
    endif()

else()
    message(STATUS "MHE generated code NOT found — run ocp_generation/generate_mhe_ocp.py first")
endif()

# ── Translational-only MHE solver (10D, no quaternion) ───────────────────────
set(MHE_TRANS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code_mhe_trans)
if(EXISTS ${MHE_TRANS_DIR}/acados_solver_quadrotor_mhe_trans.c)
    file(GLOB MHE_TRANS_MODEL  ${MHE_TRANS_DIR}/quadrotor_mhe_trans_model/*.c)
    file(GLOB MHE_TRANS_COST   ${MHE_TRANS_DIR}/quadrotor_mhe_trans_cost/*.c)
    file(GLOB MHE_TRANS_CONSTR ${MHE_TRANS_DIR}/quadrotor_mhe_trans_constraints/*.c)
    add_library(mhe_trans_solver STATIC
        ${MHE_TRANS_DIR}/acados_solver_quadrotor_mhe_trans.c
        ${MHE_TRANS_MODEL} ${MHE_TRANS_COST} ${MHE_TRANS_CONSTR})
    target_include_directories(mhe_trans_solver PUBLIC
        ${MHE_TRANS_DIR} ${ACADOS_INCLUDE_DIR} ${ACADOS_INCLUDE_DIR}/acados
        ${ACADOS_INCLUDE_DIR}/blasfeo/include ${ACADOS_INCLUDE_DIR}/hpipm/include)
    target_link_libraries(mhe_trans_solver PUBLIC
        ${ACADOS_LIB_DIR}/libacados.so ${ACADOS_LIB_DIR}/libblasfeo.so ${ACADOS_LIB_DIR}/libhpipm.so m)
    add_executable(mhe_trans_replay src/mhe/mhe_trans_replay.cpp src/mhe/mhe_trans_controller.cpp)
    target_link_libraries(mhe_trans_replay quadrotor_common mhe_trans_solver)
    message(STATUS "mhe_trans (translational-only) configured")
endif()

# ── MHE-D solver (9D, mass=param — EXP-2 disturbance-only) ───────────────────
set(MHE_TRANS_D_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code_mhe_trans_d)
if(EXISTS ${MHE_TRANS_D_DIR}/acados_solver_quadrotor_mhe_trans_d.c)
    file(GLOB MHE_TRANS_D_MODEL  ${MHE_TRANS_D_DIR}/quadrotor_mhe_trans_d_model/*.c)
    file(GLOB MHE_TRANS_D_COST   ${MHE_TRANS_D_DIR}/quadrotor_mhe_trans_d_cost/*.c)
    file(GLOB MHE_TRANS_D_CONSTR ${MHE_TRANS_D_DIR}/quadrotor_mhe_trans_d_constraints/*.c)
    add_library(mhe_trans_d_solver STATIC
        ${MHE_TRANS_D_DIR}/acados_solver_quadrotor_mhe_trans_d.c
        ${MHE_TRANS_D_MODEL} ${MHE_TRANS_D_COST} ${MHE_TRANS_D_CONSTR})
    target_include_directories(mhe_trans_d_solver PUBLIC
        ${MHE_TRANS_D_DIR} ${ACADOS_INCLUDE_DIR} ${ACADOS_INCLUDE_DIR}/acados
        ${ACADOS_INCLUDE_DIR}/blasfeo/include ${ACADOS_INCLUDE_DIR}/hpipm/include)
    target_link_libraries(mhe_trans_d_solver PUBLIC
        ${ACADOS_LIB_DIR}/libacados.so ${ACADOS_LIB_DIR}/libblasfeo.so ${ACADOS_LIB_DIR}/libhpipm.so m)
    message(STATUS "mhe_trans_d (disturbance-only, 9D) configured")
endif()

# ── NMPC-MHE solver (18D, thrust-as-input) ────────────────────────────────────
set(MHE_NMPC_CODEGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code_mhe_nmpc)
if(EXISTS ${MHE_NMPC_CODEGEN_DIR}/acados_solver_quadrotor_mhe_nmpc.c)
    message(STATUS "NMPC-MHE generated code found at ${MHE_NMPC_CODEGEN_DIR}")

    file(GLOB MHE_NMPC_MODEL_SRC  ${MHE_NMPC_CODEGEN_DIR}/quadrotor_mhe_nmpc_model/*.c)
    file(GLOB MHE_NMPC_COST_SRC   ${MHE_NMPC_CODEGEN_DIR}/quadrotor_mhe_nmpc_cost/*.c)
    file(GLOB MHE_NMPC_CONSTR_SRC ${MHE_NMPC_CODEGEN_DIR}/quadrotor_mhe_nmpc_constraints/*.c)

    add_library(mhe_nmpc_solver STATIC
        ${MHE_NMPC_CODEGEN_DIR}/acados_solver_quadrotor_mhe_nmpc.c
        ${MHE_NMPC_MODEL_SRC}
        ${MHE_NMPC_COST_SRC}
        ${MHE_NMPC_CONSTR_SRC}
    )
    target_include_directories(mhe_nmpc_solver PUBLIC
        ${MHE_NMPC_CODEGEN_DIR}
        ${ACADOS_INCLUDE_DIR}
        ${ACADOS_INCLUDE_DIR}/acados
        ${ACADOS_INCLUDE_DIR}/blasfeo/include
        ${ACADOS_INCLUDE_DIR}/hpipm/include
    )
    target_link_libraries(mhe_nmpc_solver PUBLIC
        ${ACADOS_LIB_DIR}/libacados.so
        ${ACADOS_LIB_DIR}/libblasfeo.so
        ${ACADOS_LIB_DIR}/libhpipm.so
        m
    )

    # ── Standalone 18D-MHE test (no ROS2/MuJoCo) ─────────────────────
    add_executable(mhe_nmpc_test
        src/mhe/mhe_nmpc_test.cpp
        src/mhe/mhe_nmpc_controller.cpp
    )
    target_link_libraries(mhe_nmpc_test
        quadrotor_common
        mhe_nmpc_solver
    )

    # ── Offline MHE replay on real recorded data (no ROS2/MuJoCo) ────
    add_executable(mhe_replay
        src/mhe/mhe_replay.cpp
        src/mhe/mhe_nmpc_controller.cpp
    )
    target_link_libraries(mhe_replay
        quadrotor_common
        mhe_nmpc_solver
    )

    # ── NMPC-MHE adaptive SiL (needs NMPC + 18D-MHE solvers) ─────────
    if(HAS_ROS2 AND TARGET nmpc_solver)
        add_executable(nmpc_mhe_sil
            src/nmpc/nmpc_mhe_sil.cpp
            src/nmpc/nmpc_controller.cpp
            src/mhe/mhe_trans_controller.cpp
        )
        target_link_libraries(nmpc_mhe_sil
            quadrotor_common
            quadrotor_trajectory
            nmpc_solver
            mhe_trans_solver
            mujoco_interface
        )
        ament_target_dependencies(nmpc_mhe_sil
            rclcpp nav_msgs sensor_msgs mujoco_ros_utils std_msgs std_srvs geometry_msgs quadrotor_msgs
        )
        message(STATUS "nmpc_mhe_sil target configured")
    else()
        message(STATUS "nmpc_mhe_sil: skipped (needs ROS2 + nmpc_solver)")
    endif()
else()
    message(STATUS "NMPC-MHE generated code NOT found — run ocp_generation/generate_mhe_nmpc_ocp.py first")
endif()

# ── Install targets (for colcon) ─────────────────────────────────────────────
if(ament_cmake_FOUND)
    set(_install_targets "")
    foreach(_t nmpc_sil nmpc_sim mhe_mpcc_sil nmpc_mhe_sil)
        if(TARGET ${_t})
            list(APPEND _install_targets ${_t})
        endif()
    endforeach()
    if(_install_targets)
        install(TARGETS ${_install_targets}
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()

    # Config YAML (para --params-file)
    install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
    )

    ament_package()
endif()
