project(demo)

file(GLOB ${PROJECT_NAME}_headers "*.hpp")
file(GLOB ${PROJECT_NAME}_sources "*.cpp")

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_headers} ${${PROJECT_NAME}_sources})
target_link_libraries(${PROJECT_NAME} libsolver)