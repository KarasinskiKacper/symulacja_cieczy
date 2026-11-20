find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
find_package(glm REQUIRED)

file(GLOB_RECURSE VISUALIZATION_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*
)
file(GLOB_RECURSE VISUALIZATION_HEADERS CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/include/*
)

add_library(visualization STATIC ${VISUALIZATION_SOURCES} ${VISUALIZATION_HEADERS})

target_include_directories(visualization
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(visualization
        PUBLIC
            sfml-graphics
            sfml-window
            sfml-system
            glm::glm
        PRIVATE
            config
)
