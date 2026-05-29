# 包含交叉编译工具链
include(cross.cmake)

# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.5.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wall")  # 对于 C++ 编译器
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")      # 对于 C 编译器

# 获取当前 CMakeLists.txt 文件所在目录的完整路径
set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})

# 获取上上级目录的完整路径
get_filename_component(PARENT_PARENT_DIR ${CURRENT_DIR}/.. ABSOLUTE)

# 从完整路径中提取文件夹名称
get_filename_component(PARENT_PARENT_FOLDER_NAME ${PARENT_PARENT_DIR} NAME)

# 打印上上级文件夹名称
message("Parent parent folder name: ${PARENT_PARENT_FOLDER_NAME}")

# 定义项目名称和版本，并指定使用C和C++语言
project(${PARENT_PARENT_FOLDER_NAME} VERSION 0.1.0 LANGUAGES C CXX)

# 设置OpenCV的安装路径
set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build")

# 查找OpenCV库，确保安装了所需的依赖
find_package(OpenCV REQUIRED)
find_package(PkgConfig)

# 优先使用工程内自带 NCNN 静态库；若不存在，再尝试 pkg-config 和系统路径
set(NCNN_INCLUDE_DIRS "")
set(NCNN_LIBRARIES "")
set(NCNN_LOCAL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../libraries/zf_components/ncnn")

if(EXISTS "${NCNN_LOCAL_ROOT}/include/ncnn/net.h" AND EXISTS "${NCNN_LOCAL_ROOT}/lib/libncnn.a")
    set(NCNN_INCLUDE_DIRS "${NCNN_LOCAL_ROOT}/include")
    link_directories("${NCNN_LOCAL_ROOT}/lib")
    set(NCNN_LIBRARIES ncnn)
    message(STATUS "Found NCNN via bundled project library")
elseif(PkgConfig_FOUND)
    pkg_check_modules(NCNN QUIET ncnn)
endif()

if(NCNN_FOUND)
    message(STATUS "Found NCNN via pkg-config")
    set(NCNN_INCLUDE_DIRS ${NCNN_INCLUDE_DIRS})
    set(NCNN_LIBRARIES ${NCNN_LINK_LIBRARIES})
elseif(NOT NCNN_LIBRARIES)
    find_path(NCNN_INCLUDE_DIR
        NAMES ncnn/net.h
        PATHS
            /opt/ls_2k0300_env/ncnn/include
            /opt/ls_2k0300_env/ncnn_build/include
            /usr/local/include
            /usr/include
    )

    find_library(NCNN_LIBRARY
        NAMES ncnn libncnn lqncnn
        PATHS
            /opt/ls_2k0300_env/ncnn/lib
            /opt/ls_2k0300_env/ncnn_build/lib
            /usr/local/lib
            /usr/lib
            /usr/lib64
    )

    if(NCNN_INCLUDE_DIR AND NCNN_LIBRARY)
        set(NCNN_INCLUDE_DIRS ${NCNN_INCLUDE_DIR})
        set(NCNN_LIBRARIES ${NCNN_LIBRARY})
        message(STATUS "Found NCNN via fallback search")
    else()
        message(FATAL_ERROR "未找到 NCNN。优先把 ncnn 库放到 libraries/zf_components/ncnn（含 include 与 lib/libncnn.a），否则请检查系统中的 ncnn 安装路径。")
    endif()
endif()

# 包含OpenCV和NCNN的头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${NCNN_INCLUDE_DIRS})
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
message(STATUS "NCNN_INCLUDE_DIRS: ${NCNN_INCLUDE_DIRS}")
message(STATUS "NCNN_LIBRARIES: ${NCNN_LIBRARIES}")

# 包含项目的自定义库路径
include_directories(../user)
include_directories(../code)
include_directories(../../libraries/zf_common)
include_directories(../../libraries/zf_device)
include_directories(../../libraries/zf_driver)
include_directories(../../libraries/zf_components)

# 从目录下查找C/C++文件
aux_source_directory(../user DIR_SRCS)
aux_source_directory(../code DIR_SRCS)
aux_source_directory(../../libraries/zf_common DIR_SRCS)
aux_source_directory(../../libraries/zf_device DIR_SRCS)
aux_source_directory(../../libraries/zf_driver DIR_SRCS)
aux_source_directory(../../libraries/zf_components DIR_SRCS)

# 创建可执行文件，使用项目名称作为目标名称
add_executable(${PROJECT_NAME} ${DIR_SRCS})

# 链接OpenCV和NCNN库到可执行文件
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${NCNN_LIBRARIES})