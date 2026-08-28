cmake_minimum_required(VERSION 3.10)
add_library(fast-akaze STATIC
    akaze.cpp
    AKAZEFeatures.cpp
    fed.cpp
    nldiffusion_functions.cpp
)
