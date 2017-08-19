cmake_minimum_required(VERSION 2.8)
add_library(fast-akaze STATIC
    akaze.cpp
    AKAZEFeatures.cpp
    fed.cpp
    nldiffusion_functions.cpp
)
