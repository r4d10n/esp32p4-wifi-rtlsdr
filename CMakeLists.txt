cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS components)
set(EXCLUDE_COMPONENTS rtltcp websdr dsp)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32p4-fm-radio-mono)
