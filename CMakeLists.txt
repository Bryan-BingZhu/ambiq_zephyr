#
# Copyright (c) 2023 Ambiq Micro, Inc.
# All rights reserved.
#

# Point to Ambiq root directory.
set(AMBIQ_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "Ambiq root directory")

# We need to cache the build type so that it is not out of sync when CONF_FILE
# is cached.
set(CONF_FILE_BUILD_TYPE ${CONF_FILE_BUILD_TYPE} CACHE INTERNAL "The build type")

add_subdirectory(samples)
add_subdirectory(drivers)
