cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(assignment_ses)

target_include_directories(app PRIVATE inc)
target_sources(app PRIVATE src/main.c src/robot.c src/robot/sensors/tap_detect.c 
    src/robot/sensors/lsm6dsox.c src/robot/power.c src/robot/sensors/lis3mdl.c 
    src/robot/calibration.c src/robot/settings.c src/robot/sensors/crash_detect.c src/robot/sensors/int1_gpio.c)
