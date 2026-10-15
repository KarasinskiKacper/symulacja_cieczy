find_package(glm REQUIRED)

file(GLOB_RECURSE PHYSICS_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*
)
file(GLOB_RECURSE PHYSICS_HEADERS CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/include/*
)

add_library(physics STATIC ${PHYSICS_SOURCES} ${PHYSICS_HEADERS})

target_include_directories(physics
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(physics
        PUBLIC glm::glm
        PRIVATE config
)
