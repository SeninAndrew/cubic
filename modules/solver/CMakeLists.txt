project(libsolver)

file(GLOB ${PROJECT_NAME}_headers "include/solver/*.hpp")
file(GLOB ${PROJECT_NAME}_sources "src/*.cpp")
file(GLOB ${PROJECT_NAME}_tests "tests/*.hpp" "tests/*.cpp")

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_headers} ${${PROJECT_NAME}_sources})
add_executable(${PROJECT_NAME}_test ${${PROJECT_NAME}_tests})

include_directories(include/solver/)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_test gtest ${PROJECT_NAME})