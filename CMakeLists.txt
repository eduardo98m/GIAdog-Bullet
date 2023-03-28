cmake_minimum_required(VERSION 3.0)

PROJECT(Example)
if (MSVC)
    #add_compile_options(/W4 /WX)
    add_compile_options(/Zc:__cplusplus)
    add_compile_options(/std:c++latest)
    add_compile_options(/permissive-)
    add_compile_options(/MP)

    # Set _ITERATOR_DEBUG_LEVEL=2 for Debug configuration
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MD")

    # Set /MD for Release configuration
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
endif()

SET(BULLET_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/bullet3/build_cmake/local_install/include/bullet)
SET(BULLET_LIBRARY_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/bullet3/build_cmake/local_install/lib)



if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    SET(CPP_BULLET_LIB ${BULLET_LIBRARY_DIRS}/Debug/CPPBullet.lib)
else()
    SET(CPP_BULLET_LIB ${BULLET_LIBRARY_DIRS}/Release/CPPBullet.lib)
endif()



Include_directories(${BULLET_INCLUDE_DIR})
add_executable(Example example2.cpp)



target_link_libraries(Example ${CPP_BULLET_LIB} opengl32)

