cmake_minimum_required(VERSION 3.12)
project(quadrf_phasegaze C)

add_compile_options(-Wall -Wextra -O3 -ffast-math)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    add_compile_options(-mcpu=cortex-a76 -mtune=cortex-a76)
endif()

find_library(FFTW3F_LIB fftw3f REQUIRED)
find_package(Threads REQUIRED)

add_executable(quadrf-phasegaze
    src/main.c
    src/csi_dev.c
    src/dsp.c
    src/server.c
    src/external/mongoose.c
)

target_include_directories(quadrf-phasegaze PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# mongoose is third-party; keep it out of the Cortex-A76 / LTO flags
set_source_files_properties(src/external/mongoose.c PROPERTIES
    COMPILE_FLAGS "-O2 -Wno-unused-parameter"
)

target_link_libraries(quadrf-phasegaze PRIVATE
    ${FFTW3F_LIB}
    Threads::Threads
    m
)

include(GNUInstallDirs)

install(TARGETS quadrf-phasegaze DESTINATION ${CMAKE_INSTALL_BINDIR})
install(PROGRAMS packaging/quadrf-phasegaze-launch DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY web/ DESTINATION ${CMAKE_INSTALL_DATADIR}/quadrf/phasegaze)
install(FILES packaging/quadrf-phasegaze.json DESTINATION ${CMAKE_INSTALL_DATADIR}/quadrf/apps.d)
install(FILES packaging/com.scalerf.QuadRF.PhaseGaze.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications)
install(FILES packaging/quadrf-phasegaze.svg DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps)
install(FILES packaging/com.scalerf.QuadRF.PhaseGaze.metainfo.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/metainfo)
install(FILES packaging/quadrf-phasegaze.1 DESTINATION ${CMAKE_INSTALL_MANDIR}/man1)
install(PROGRAMS packaging/46-phasegaze DESTINATION lib/quadrf/apply.d)
