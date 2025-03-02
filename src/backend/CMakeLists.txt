cmake_minimum_required(VERSION 3.10)

project(CommunicationHandlerProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_policy(SET CMP0167 NEW)

find_package(PostgreSQL REQUIRED)
find_package(cpprestsdk REQUIRED)
find_package(nlohmann_json 3.11.3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS random system thread filesystem chrono atomic date_time regex)

set(SOURCES
        main.cpp
        module/CommunicationHandler.cpp
        module/CommunicationHandler.h
        core/LineBuilder.cpp
        core/LineBuilder.h
        core/PointMath.cpp
        core/PointMath.h
        core/Airplane.cpp
        core/Airplane.h
)

add_executable(communication_handler ${SOURCES})
file(COPY assets DESTINATION ${CMAKE_BINARY_DIR})

target_include_directories(communication_handler PRIVATE ${PostgreSQL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(communication_handler PRIVATE
        cpprestsdk::cpprest
        OpenSSL::SSL
        OpenSSL::Crypto
        ${Boost_LIBRARIES}
        ${PostgreSQL_LIBRARIES}
        nlohmann_json::nlohmann_json
)

MESSAGE(STATUS "Boost Libs" ${Boost_LIBRARIES})

set_target_properties(communication_handler PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH_USE_LINK_PATH TRUE
)
