
# check that the project is set up correctly
if((NOT DEFINED MBED_CMAKE_VERSION) AND (NOT DEFINED MBED_APP_JSON_PATH))
    message(FATAL_ERROR "This repo is only set up to be included in an mbed-cmake or mbed-ce build system")
endif()

if(NOT DEFINED FLASHLOG_CONFIG_SOURCES)
    message(FATAL_ERROR "Please define FLASHLOG_CONFIG_SOURCES in cmake to point to FlashLogConfig.h and any additional source files")
endif()

set(FLASHLOG_SOURCES
    src/FlashLog.cpp
    include/FlashLog.h
    src/Packet.cpp
    include/Packet.h
    ${FLASHLOG_CONFIG_SOURCES})

set(TEST_FLASH_SOURCES
    testsuite/FlashLogTestSuite.h
    testsuite/FlashLogTestSuite.cpp)

add_library(flashlog STATIC ${FLASHLOG_SOURCES})
target_link_libraries(flashlog mbed-os ${FLASHLOG_EXTRA_LIBS})
target_include_directories(flashlog PUBLIC include)

if(DEFINED MBED_APP_JSON_PATH)
    if(NOT TARGET mbed-storage-sd)
        message(FATAL_ERROR "For Mbed CE, FlashLog requires '\"target.components_add\" : [\"SD\"]' in mbed_app.json")
    endif()

    target_link_libraries(flashlog mbed-storage-sd)
endif()

# disable warnings that aren't relevant here
target_compile_options(flashlog PRIVATE -Wno-type-limits)

# include directory where the config header is
list(GET FLASHLOG_CONFIG_SOURCES 0 FLASHLOG_CONFIG_HEADER)
get_filename_component(CONFIG_HEADER_DIR ${FLASHLOG_CONFIG_HEADER} DIRECTORY)
target_include_directories(flashlog PUBLIC ${CONFIG_HEADER_DIR})

if(DEFINED MBED_CMAKE_VERSION)
    # mbed-cmake version
    add_mbed_executable(test_flash ${TEST_FLASH_SOURCES})
else()
    # mbed-ce version
    add_executable(test_flash ${TEST_FLASH_SOURCES})
    mbed_set_post_build(test_flash)
endif()
target_link_libraries(test_flash flashlog ${FLASHLOG_TEST_HARNESS_LIBRARIES})
target_compile_options(test_flash PRIVATE -Wno-unknown-pragmas)

target_compile_features(test_flash PRIVATE cxx_std_17)
