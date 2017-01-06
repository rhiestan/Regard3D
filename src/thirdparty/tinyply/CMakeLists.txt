cmake_minimum_required(VERSION 2.6)
project(tinyply)

SET(ALL_SRC tinyply.cpp)

SET(ALL_HEADERS tinyply.h)

ADD_LIBRARY(tinyply STATIC ${ALL_SRC} ${ALL_HEADERS})
