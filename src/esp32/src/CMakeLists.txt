# Explicit source collection to keep active code layout clear.
# Active groups:
#   - root entry/planner sources
#   - comm/ communication subsystem
#   - motion/ stepper and executor subsystem

file(GLOB root_sources
	${CMAKE_SOURCE_DIR}/src/main.cpp
)

file(GLOB comm_sources ${CMAKE_SOURCE_DIR}/src/comm/*.cpp)
file(GLOB motion_sources ${CMAKE_SOURCE_DIR}/src/motion/*.cpp)

# Legacy duplicates are kept in tree for compatibility only and must not build.
list(REMOVE_ITEM comm_sources ${CMAKE_SOURCE_DIR}/src/comm/multi_axis_executor.cpp)

set(app_sources ${root_sources} ${comm_sources} ${motion_sources})

idf_component_register(SRCS ${app_sources})
