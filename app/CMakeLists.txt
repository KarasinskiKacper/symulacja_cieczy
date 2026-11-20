cmake_minimum_required(VERSION 4.3)
project(symulacja_cieczy)

set(CMAKE_CXX_STANDARD 26)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(
            -Wall
            -Wextra
            -Wpedantic
            -Wshadow
            -Wconversion
            -Wsign-conversion
            -Wdouble-promotion
            -Wold-style-cast
            -Wuseless-cast
            -Wcast-align
            -Wnon-virtual-dtor
            -Woverloaded-virtual
            -Wnull-dereference
            -Wformat=2
            -Wduplicated-cond
            -Wduplicated-branches
            -Wlogical-op
    )
endif()

add_subdirectory(config)
add_subdirectory(physics)
add_subdirectory(visualization)

set(SOURCES main.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})

find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME} PRIVATE
        config
        physics
        visualization
        Threads::Threads
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/../assets $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets
)
