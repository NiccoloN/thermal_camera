cmake_minimum_required(VERSION 3.16)

function(make_absolute_path PATH_VAR)
    get_filename_component(ABS_PATH "${${PATH_VAR}}" REALPATH)
    set(${PATH_VAR} "${ABS_PATH}" PARENT_SCOPE)
endfunction()

set(MIOSIX_KPATH ${CMAKE_CURRENT_LIST_DIR}/miosix-kernel/miosix CACHE PATH "Path to the miosix kernel")
make_absolute_path(MIOSIX_KPATH)

set(CMAKE_TOOLCHAIN_FILE ${MIOSIX_KPATH}/cmake/Toolchains/gcc.cmake CACHE PATH "Path to your toolchain file")
make_absolute_path(CMAKE_TOOLCHAIN_FILE)

set(ACPP_DIR ${CMAKE_CURRENT_LIST_DIR}/AdaptiveCpp-Embedded/install-gcc-mcu CACHE PATH "Path to AdaptiveCpp")
make_absolute_path(ACPP_DIR)

set(MIOSIX_LLVM_PATH /home/nico/miosix/compilers/llvm-18 CACHE PATH "Path to the Miosix LLVM compiler")
make_absolute_path(MIOSIX_LLVM_PATH)

set(MIOSIX_GCC_PATH /home/nico/miosix/compilers/arm-miosix-eabi CACHE PATH "Path to the Miosix GCC compiler")
make_absolute_path(MIOSIX_GCC_PATH)

set(OPENMP gomp CACHE STRING "OpenMP library to use")
set(THERMAL_CAMERA_FRAME_PROCESS_VARIANT "openmp" CACHE STRING "MLX90640 frame processing backend")
set_property(CACHE THERMAL_CAMERA_FRAME_PROCESS_VARIANT PROPERTY STRINGS serial openmp sycl)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build")
set(CMAKE_CXX_STANDARD 17 CACHE STRING "Project C++ standard")

project(ThermalCamera C CXX ASM)

set(MIOSIX_CONFIG_PATH ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "Include directory for miosix config")
set(MIOSIX_OPT_BOARD rp2040_raspberry_pi_pico CACHE STRING "Target board")
set(MIOSIX_CUSTOM_BOARD_SETTINGS_INC
    ${CMAKE_CURRENT_LIST_DIR}/config/arch/cortexM0plus_rp2040/rp2040_raspberry_pi_pico
    CACHE PATH "Include directory for custom board_settings.h" FORCE)

add_subdirectory(${MIOSIX_KPATH} ${CMAKE_BINARY_DIR}/miosix EXCLUDE_FROM_ALL)
include(LinkTarget)

set(MXGUI_SRCS
    mxgui/display.cpp
    mxgui/font.cpp
    mxgui/misc_inst.cpp
    mxgui/tga_image.cpp
    mxgui/textbox.cpp
    mxgui/level2/input.cpp
    mxgui/level2/application.cpp
    mxgui/level2/drawing_context_proxy.cpp
    mxgui/level2/label.cpp
    mxgui/level2/button.cpp
    mxgui/level2/checkbox.cpp
    mxgui/level2/radio_button.cpp
    mxgui/level2/scrolling_list.cpp
    mxgui/level2/simple_plot.cpp
    mxgui/drivers/display_stm3210e-eval.cpp
    mxgui/drivers/event_stm3210e-eval.cpp
    mxgui/drivers/display_mp3v2.cpp
    mxgui/drivers/event_mp3v2.cpp
    mxgui/drivers/display_strive.cpp
    mxgui/drivers/display_st7735.cpp
    mxgui/drivers/display_oledboard2.cpp
    mxgui/drivers/event_strive.cpp
    mxgui/drivers/display_redbull_v2.cpp
    mxgui/drivers/event_redbull_v2.cpp
    mxgui/drivers/display_bitsboard.cpp
    mxgui/drivers/display_sony-newman.cpp
    mxgui/drivers/event_sony-newman.cpp
    mxgui/drivers/display_stm32f4discovery.cpp
    mxgui/drivers/event_stm32f4discovery.cpp
    mxgui/drivers/display_generic_1bpp.cpp
    mxgui/drivers/display_st25dvdiscovery.cpp
    mxgui/drivers/display_stm3220g-eval.cpp
    mxgui/drivers/event_stm3220g-eval.cpp
    mxgui/drivers/event_st25dvdiscovery.cpp
)

add_library(mxgui STATIC ${MXGUI_SRCS})
target_include_directories(mxgui PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/mxgui
)
target_compile_definitions(mxgui PRIVATE MXGUI_LIBRARY)
target_link_libraries(mxgui PUBLIC miosix)

set(APP_SRCS
    main.cpp
    application.cpp
    renderer.cpp
    colormap.cpp
    version.cpp
    drivers/display_er_oledm015.cpp
    drivers/misc.cpp
    drivers/options_save.cpp
    drivers/mlx90640.cpp
    drivers/mlx90640frame.cpp
    drivers/MLX90640_API.cpp
    drivers/rp2040_spi.cpp
    drivers/rp2040_i2c.cpp
    drivers/arm_pl022_spi.cpp
    images/batt0icon.cpp
    images/batt25icon.cpp
    images/batt50icon.cpp
    images/batt75icon.cpp
    images/batt100icon.cpp
    images/miosixlogoicon.cpp
    images/emissivityicon.cpp
    images/smallcelsiusicon.cpp
    images/largecelsiusicon.cpp
    images/pauseicon.cpp
    images/usbicon.cpp
)

add_executable(main ${APP_SRCS})
target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/mxgui
)
target_compile_options(main PRIVATE
    -ffast-math
    -fdata-sections
)

if(THERMAL_CAMERA_FRAME_PROCESS_VARIANT STREQUAL "serial")
    set(THERMAL_CAMERA_FRAME_PROCESS_VARIANT_VALUE 0)
elseif(THERMAL_CAMERA_FRAME_PROCESS_VARIANT STREQUAL "openmp")
    set(THERMAL_CAMERA_FRAME_PROCESS_VARIANT_VALUE 1)
elseif(THERMAL_CAMERA_FRAME_PROCESS_VARIANT STREQUAL "sycl")
    set(THERMAL_CAMERA_FRAME_PROCESS_VARIANT_VALUE 2)
else()
    message(FATAL_ERROR "Unsupported THERMAL_CAMERA_FRAME_PROCESS_VARIANT=${THERMAL_CAMERA_FRAME_PROCESS_VARIANT}")
endif()

target_compile_definitions(main PRIVATE
    THERMAL_CAMERA_FRAME_PROCESS_VARIANT=${THERMAL_CAMERA_FRAME_PROCESS_VARIANT_VALUE}
)

if(NOT THERMAL_CAMERA_FRAME_PROCESS_VARIANT STREQUAL "serial")
    target_compile_options(main PRIVATE -fopenmp)
endif()

if(THERMAL_CAMERA_FRAME_PROCESS_VARIANT STREQUAL "sycl")
    target_include_directories(main PRIVATE
        ${ACPP_DIR}/include/AdaptiveCpp
    )
    target_compile_definitions(main PRIVATE
        __OPENSYCL__
        __HIPSYCL__
        __ADAPTIVECPP__
        __ACPP__
        __ACPP_ENABLE_OMPHOST_TARGET__
    )
    target_compile_options(main PRIVATE
        -D_ENABLE_EXTENDED_ALIGNED_STORAGE
        -DHIPSYCL_NO_FIBERS
    )
    target_link_directories(main PRIVATE
        ${ACPP_DIR}/lib
        ${ACPP_DIR}/lib/hipSYCL
    )
endif()

target_link_libraries(main PRIVATE
    mxgui
    -Wl,--start-group
    miosix
    -Wl,--end-group
)

if(THERMAL_CAMERA_FRAME_PROCESS_VARIANT STREQUAL "sycl")
    target_link_libraries(main PRIVATE
        -Wl,--start-group
        -lacpp-rt
        -lrt-backend-omp
        -Wl,--end-group
    )
endif()

if(NOT THERMAL_CAMERA_FRAME_PROCESS_VARIANT STREQUAL "serial")
    target_link_libraries(main PRIVATE ${OPENMP})
endif()

miosix_link_target(main)
