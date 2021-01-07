
# check that the project is set up correctly
if(NOT DEFINED MBED_CMAKE_VERSION)
    message(FATAL_ERROR "This repo is only set up to be included in an mbed-cmake build system")
endif()

if(NOT DEFINED FLASHLOG_CONFIG_HEADER)
    message(FATAL_ERROR "Please define FLASHLOG_CONFIG_HEADER in cmake to point to FlashLogConfig.h")
endif()

set(FLASHLOG_SOURCES
    src/FlashLog.cpp
    include/FlashLog.h
    include/Packet.h
    ${FLASHLOG_CONFIG_HEADER})

set(TEST_FLASH_SOURCES
    testsuite/FlashLogTestSuite.h
    testsuite/FlashLogTestSuite.cpp)

add_library(flashlog STATIC ${FLASHLOG_SOURCES})
target_link_libraries(flashlog mbed-os)
target_include_directories(flashlog PUBLIC include)

# disable a warning that isn't relevant here
target_compile_options(flashlog PRIVATE -Wno-type-limits)

# include directory where the config header is
get_filename_component(CONFIG_HEADER_DIR ${FLASHLOG_CONFIG_HEADER} DIRECTORY)
target_include_directories(flashlog PUBLIC ${CONFIG_HEADER_DIR})

add_mbed_executable(test_flash ${TEST_FLASH_SOURCES})
target_link_libraries(test_flash flashlog)
