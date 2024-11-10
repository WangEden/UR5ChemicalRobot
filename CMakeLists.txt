cmake_minimum_required(VERSION 3.5)

project(App VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# find_package(PkgConfig REQUIRED)

# # Temporarily modify PKG_CONFIG_PATH for this find_package call
# set(LOCAL_PKG_CONFIG_PATH "${CMAKE_SOURCE_DIR}/modules/lib/pkgconfig")
# if (NOT DEFINED ENV{PKG_CONFIG_PATH})
#     set(ENV{PKG_CONFIG_PATH} ${LOCAL_PKG_CONFIG_PATH})
# else()
#     set(ENV{PKG_CONFIG_PATH} ${LOCAL_PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})
# endif()
# pkg_check_modules(REALSENSE2 REQUIRED realsense2)

# Find the specific library
# message(STATUS "PKG_CONFIG_PATH: $ENV{PKG_CONFIG_PATH}")
# message(STATUS "REALSENSE2_INCLUDE_DIRS: ${REALSENSE2_INCLUDE_DIRS}")
# message(STATUS "REALSENSE2_LIBRARY_DIRS: ${REALSENSE2_LIBRARY_DIRS}")
# message(STATUS "REALSENSE2_LIBRARIES: ${REALSENSE2_LIBRARIES}")
# 设置 CMake 模块路径
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_SOURCE_DIR}/modules/lib/cmake")
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets LinguistTools SerialPort REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets LinguistTools SerialPort REQUIRED)
# find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(ur_client_library REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Optionally reset PKG_CONFIG_PATH if needed
# set(ENV{PKG_CONFIG_PATH} ${ORIGINAL_PKG_CONFIG_PATH})

set(REALSENSE2_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/modules/include")
set(REALSENSE2_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/modules/lib")
set(REALSENSE2_LIBRARIES "realsense2")

set(TS_FILES settings/App_zh_CN.ts)

set(PROJECT_SOURCES
    src/main.cpp
    src/Widget/widget.cpp include/widget.h
    src/Device/serialport.cpp include/serialport.h
    src/Device/d435i.cpp include/d435i.h
    src/Device/RS485.cpp include/RS485.h
    src/Device/UR5.cpp include/UR5.h
    src/Widget/CameraWidget.cpp include/CameraWidget.h
    src/Thread/CameraThread.cpp include/CameraThread.h
    src/Function/Function.cpp include/Function.h
    src/Thread/UR5Thread.cpp include/UR5Thread.h
    src/Function/Calculation.cpp include/Calculation.h
    ${TS_FILES}
)

add_executable(calibrate)
target_sources(calibrate 
PRIVATE 
    src/Calibrate/calibrate.cpp
    src/Device/d435i.cpp
    src/Device/UR5.cpp
    src/Function/Calculation.cpp include/Calculation.h
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(App
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET App APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    if(ANDROID)
        add_library(App SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(App
            ${PROJECT_SOURCES}
        )
    endif()

    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

target_include_directories(App 
PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_iNCLUDE_DIRS}
    ${ur_client_library_INCLUDE_DIRS}
    ${REALSENSE2_INCLUDE_DIR}
    # /usr/local/include
)
target_include_directories(calibrate
PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_iNCLUDE_DIRS}
    ${ur_client_library_INCLUDE_DIRS}
    ${REALSENSE2_INCLUDE_DIR}
    # /usr/local/include
)

target_link_directories(App
PUBLIC
    ${CMAKE_SOURCE_DIR}/libs
    ${CMAKE_SOURCE_DIR}/modules/lib
    ${REALSENSE2_LIBRARY_DIR}
    # /usr/local/lib
)
target_link_directories(calibrate
PUBLIC
    ${CMAKE_SOURCE_DIR}/libs
    ${CMAKE_SOURCE_DIR}/modules/lib
    ${REALSENSE2_LIBRARY_DIR}
    # /usr/local/lib
)

target_link_libraries(App 
PRIVATE 
    Qt${QT_VERSION_MAJOR}::Widgets 
    ${Qt5SerialPort_LIBRARIES}
    ${OpenCV_LIBS} 
    # realsense2  
    Threads::Threads 
    ${Boost_LIBRARIES} 
    ${ur_client_library_LIBRARIES}
    ${REALSENSE2_LIBRARIES}
    Eigen3::Eigen
)
target_link_libraries(calibrate
    ${OpenCV_LIBS} 
    # realsense2 
    Threads::Threads 
    ${Boost_LIBRARIES} 
    ${ur_client_library_LIBRARIES}
    ${REALSENSE2_LIBRARIES}
    Eigen3::Eigen
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.App)
endif()
set_target_properties(App PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS App
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(App)
endif()
