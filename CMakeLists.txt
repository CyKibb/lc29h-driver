# Find check
find_path(CHECK_INCLUDE_DIR check.h)
find_library(CHECK_LIBRARY NAMES check)

add_executable(gnss_drivers_tests ./tests/gnss_driver_tests.c ./tests/gnss_driver_tests_setup.c)
target_include_directories(gnss_drivers_tests PUBLIC ${CHECK_INCLUDE_DIR} ./tests/includes)
target_link_libraries(gnss_drivers_tests ${CHECK_LIBRARY} qc_lc29_driver)

add_library(qc_lc29_driver STATIC ./src/qc_lc29_driver.c)

target_include_directories(qc_lc29_driver PUBLIC includes)


