cmake_minimum_required(VERSION 3.5)

project(simple_navigator VERSION 0.1 LANGUAGES CXX)

set(GRAPH_TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests)
set(GRAPH_3D_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3d_party)
set(GRAPH_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(GRAPH_EXE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(GRAPH_MODEL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/model)
set(GRAPH_CONTROLLER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/controller)
set(GRAPH_VIEW_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/view)

set(GRAPH_COVERAGE_LIBS "" CACHE STRING "")
set(GRAPH_COVERAGE_FLAGS "" CACHE STRING "")
set(GRAPH_WARNING_FLAGS "-Wall -Werror -Wextra" CACHE STRING "")
set(GRAPH_STANDART_FLAGS "-std=c++17" CACHE STRING "")
set(GRAPH_FLAGS "${GRAPH_COVERAGE_FLAGS} ${GRAPH_WARNING_FLAGS} ${GRAPH_STANDART_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GRAPH_FLAGS}")

include_directories(
  ${GRAPH_SRC_DIR}
  ${GRAPH_3D_PARTY_DIR}/mtlc/include/mtlc
  ${GRAPH_3D_PARTY_DIR}/token_parser/include/token_parser
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(
  ${GRAPH_3D_PARTY_DIR}/token_parser
  token_parser
)

set(SOURCE_EXE 
  ${GRAPH_EXE_SOURCE_DIR}/main.cc
)	

set(SOURCE_GRAPH
  ${GRAPH_MODEL_SOURCE_DIR}/graph/factory_graph_exporter.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/factory_graph_exporter.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph/factory_graph_importer.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/factory_graph_importer.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph/factory_graph_imp.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/factory_graph_imp.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_edge.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_edge.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_exporter_adjacency_matrix.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_exporter_adjacency_matrix.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_exporter_dot.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_exporter_dot.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_exporter.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_importer_adjacency_matrix.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_importer_adjacency_matrix.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_importer_dot.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_importer_dot.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_importer.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_imp_adjacency_matrix.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_imp_adjacency_matrix.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph_imp.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph/graph.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph_algorithms/graph_algorithms.h
  ${GRAPH_MODEL_SOURCE_DIR}/graph_algorithms/graph_algorithms.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph_algorithms/graph_algorithms_utils.cc
  ${GRAPH_MODEL_SOURCE_DIR}/graph_algorithms/salesman_problem.cc
)

set(SOURCE_MODEL
  ${GRAPH_MODEL_SOURCE_DIR}/model.cc
  ${GRAPH_MODEL_SOURCE_DIR}/model.h
)

set(SOURCE_CONTROLLER
  ${GRAPH_CONTROLLER_SOURCE_DIR}/controller.h
  ${GRAPH_CONTROLLER_SOURCE_DIR}/controller_default.cc
  ${GRAPH_CONTROLLER_SOURCE_DIR}/controller_default.h
)

set(SOURCE_VIEW
  ${GRAPH_VIEW_SOURCE_DIR}/view.cc
  ${GRAPH_VIEW_SOURCE_DIR}/view.h
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

add_library(graph STATIC ${SOURCE_GRAPH})

add_library(model STATIC ${SOURCE_MODEL})

add_library(controller STATIC ${SOURCE_CONTROLLER})

add_library(view STATIC ${SOURCE_VIEW})

add_executable(simple_navigator ${SOURCE_EXE})	

target_link_libraries(simple_navigator
  model
  graph
  token_parser
  controller
  view
)

set(SOURCE_TEST
  ${GRAPH_TESTS_DIR}/test.cc
  ${GRAPH_TESTS_DIR}/model/graph_algoritms/bfs.cc
  ${GRAPH_TESTS_DIR}/model/graph_algoritms/dfs.cc
  ${GRAPH_TESTS_DIR}/model/graph_algoritms/shortest_path_between_all.cc
  ${GRAPH_TESTS_DIR}/model/graph_algoritms/spanning_tree.cc
  ${GRAPH_TESTS_DIR}/model/graph_algoritms/shortest_path_between_vertices.cc
  ${GRAPH_TESTS_DIR}/model/graph_algoritms/solve_salesman_problem.cc
)

add_executable(simple_navigator_test ${SOURCE_TEST})

target_link_libraries(simple_navigator_test
  model
  graph
  token_parser
  gtest
  ${GRAPH_COVERAGE_LIBS}
)
