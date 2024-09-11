cmake_minimum_required(VERSION 3.10)

project(untitled)

# Set the C++ standard to use
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add all your source files here
set(SOURCES
        main.cpp
        App.cpp
        reader.cpp
)

# Add executable target
add_executable(YourExecutable ${SOURCES})