cmake_minimum_required(VERSION 4.1.1)
project(
  shape_detector
  VERSION 0.1
  LANGUAGES CXX)

include(cmake/config.cmake)

set(SHAPE_DETECTOR_APP shape_detector)
set(RUN_SHAPE_DETECTOR_APP run_${SHAPE_DETECTOR_APP})

set(SHAPE_DETECTOR_SRC shape_detector.cc utils.cc)

add_executable(${SHAPE_DETECTOR_APP} ${SHAPE_DETECTOR_SRC})

set_target_properties(${SHAPE_DETECTOR_APP} PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                                       "${INSTALL_DIR}")

target_compile_options(${SHAPE_DETECTOR_APP} PRIVATE -Wall -Werror -Wextra)

# target_compile_definitions(${SHAPE_DETECTOR_APP} PRIVATE EDGES)
# target_compile_definitions(${SHAPE_DETECTOR_APP} PRIVATE HSV)
# target_compile_definitions(${SHAPE_DETECTOR_APP} PRIVATE HSV_PLUS_EDGES)
# target_compile_definitions(${SHAPE_DETECTOR_APP} PRIVATE MASK)
target_compile_definitions(${SHAPE_DETECTOR_APP} PRIVATE MODE_CONFIG)
# target_compile_definitions(${SHAPE_DETECTOR_APP} PRIVATE MODE_OUTPUT)

target_link_libraries(${SHAPE_DETECTOR_APP} ${OpenCV_LIBS})

add_custom_target(
  ${RUN_SHAPE_DETECTOR_APP}
  COMMAND "${INSTALL_DIR}/${SHAPE_DETECTOR_APP}"
  DEPENDS ${SHAPE_DETECTOR_APP})
