list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set_property(GLOBAL PROPERTY CMAKE_RULE_MESSAGES OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(INSTALL_DIR ${CMAKE_SOURCE_DIR}/install_d)

enable_testing()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# OpenCV
#----------------------------------------------------------→
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

