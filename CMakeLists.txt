project(SDF_fsm)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3d_party/utils)

set(UTILS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3d_party/utils/include")

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/SFD_fsm.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
	${UTILS_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
	ipv4_utils
	cl_options
)
