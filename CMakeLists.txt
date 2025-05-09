cmake_minimum_required(VERSION 3.10)
project(dk_line_op VERSION 0.1 LANGUAGES CXX)

# — 指定 C++ 标准 —
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# — 头文件搜索路径 —
# interface/ 存放公有头文件
# thirdparty/eigen/ 存放 Eigen 源码（头文件）
include_directories(
  ${PROJECT_SOURCE_DIR}/interface
  ${PROJECT_SOURCE_DIR}/thirdparty/eigen
)

# — 源文件列表 —
set(DK_LINE_SOURCES
  src/line_manager.cpp
  src/line_interface.cpp
)

# — 构建静态库 —
add_library(${PROJECT_NAME} STATIC ${DK_LINE_SOURCES})

# 对外暴露的头文件路径
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty/eigen>
)

# 安装规则
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
