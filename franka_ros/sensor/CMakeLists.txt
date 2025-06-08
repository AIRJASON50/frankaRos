cmake_minimum_required(VERSION 3.16)
project(force_sensor_reader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必要的库
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# 查找串口库
find_path(SERIAL_INCLUDE_DIR NAMES serial/serial.h)
find_library(SERIAL_LIBRARIES NAMES serial)

if(NOT SERIAL_INCLUDE_DIR OR NOT SERIAL_LIBRARIES)
    message(STATUS "Serial library not found, will use built-in implementation")
    set(USE_BUILTIN_SERIAL TRUE)
endif()

# 包含头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(src)
if(SERIAL_INCLUDE_DIR)
    include_directories(${SERIAL_INCLUDE_DIR})
endif()

# 创建主程序可执行文件
add_executable(force_sensor_reader
    src/force_sensor_reader.cpp
    src/serial_port.cpp
    src/data_visualizer.cpp
)

# 创建调试程序可执行文件
add_executable(debug_reader
    src/debug_reader.cpp
    src/serial_port.cpp
)

# 创建力矩测试程序可执行文件
add_executable(moment_test
    src/moment_test.cpp
    src/serial_port.cpp
)

# 链接库 - 主程序
target_link_libraries(force_sensor_reader
    ${OpenCV_LIBS}
    Threads::Threads
)

# 链接库 - 调试程序
target_link_libraries(debug_reader
    Threads::Threads
)

# 链接库 - 力矩测试程序
target_link_libraries(moment_test
    Threads::Threads
)

if(SERIAL_LIBRARIES)
    target_link_libraries(force_sensor_reader ${SERIAL_LIBRARIES})
    target_link_libraries(debug_reader ${SERIAL_LIBRARIES})
    target_link_libraries(moment_test ${SERIAL_LIBRARIES})
endif()

# 编译选项
target_compile_options(force_sensor_reader PRIVATE 
    -O3 
    -march=native 
    -Wall 
    -Wextra
)

target_compile_options(debug_reader PRIVATE 
    -O3 
    -march=native 
    -Wall 
    -Wextra
)

target_compile_options(moment_test PRIVATE 
    -O3 
    -march=native 
    -Wall 
    -Wextra
)

# 安装
install(TARGETS force_sensor_reader debug_reader moment_test
    RUNTIME DESTINATION bin
) 