# CMake generated Testfile for 
# Source directory: /home/claudia/ForestGuard/src/ign-utils/src
# Build directory: /home/claudia/ForestGuard/build_ign/src
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(UNIT_Environment_TEST "/home/claudia/ForestGuard/build_ign/bin/UNIT_Environment_TEST" "--gtest_output=xml:/home/claudia/ForestGuard/build_ign/test_results/UNIT_Environment_TEST.xml")
set_tests_properties(UNIT_Environment_TEST PROPERTIES  TIMEOUT "240" _BACKTRACE_TRIPLES "/usr/share/cmake/ignition-cmake2/cmake2/IgnUtils.cmake;1725;add_test;/home/claudia/ForestGuard/src/ign-utils/src/CMakeLists.txt;4;ign_build_tests;/home/claudia/ForestGuard/src/ign-utils/src/CMakeLists.txt;0;")
add_test(check_UNIT_Environment_TEST "/usr/bin/python3.10" "/usr/share/ignition/ignition-cmake2/tools/check_test_ran.py" "/home/claudia/ForestGuard/build_ign/test_results/UNIT_Environment_TEST.xml")
set_tests_properties(check_UNIT_Environment_TEST PROPERTIES  _BACKTRACE_TRIPLES "/usr/share/cmake/ignition-cmake2/cmake2/IgnUtils.cmake;1741;add_test;/home/claudia/ForestGuard/src/ign-utils/src/CMakeLists.txt;4;ign_build_tests;/home/claudia/ForestGuard/src/ign-utils/src/CMakeLists.txt;0;")
add_test(UNIT_NeverDestroyed_TEST "/home/claudia/ForestGuard/build_ign/bin/UNIT_NeverDestroyed_TEST" "--gtest_output=xml:/home/claudia/ForestGuard/build_ign/test_results/UNIT_NeverDestroyed_TEST.xml")
set_tests_properties(UNIT_NeverDestroyed_TEST PROPERTIES  TIMEOUT "240" _BACKTRACE_TRIPLES "/usr/share/cmake/ignition-cmake2/cmake2/IgnUtils.cmake;1725;add_test;/home/claudia/ForestGuard/src/ign-utils/src/CMakeLists.txt;4;ign_build_tests;/home/claudia/ForestGuard/src/ign-utils/src/CMakeLists.txt;0;")
add_test(check_UNIT_NeverDestroyed_TEST "/usr/bin/python3.10" "/usr/share/ignition/ignition-cmake2/tools/check_test_ran.py" "/home/claudia/ForestGuard/build_ign/test_results/UNIT_NeverDestroyed_TEST.xml")
set_tests_properties(check_UNIT_NeverDestroyed_TEST PROPERTIES  _BACKTRACE_TRIPLES "/usr/share/cmake/ignition-cmake2/cmake2/IgnUtils.cmake;1741;add_test;/home/claudia/ForestGuard/src/ign-utils/src/CMakeLists.txt;4;ign_build_tests;/home/claudia/ForestGuard/src/ign-utils/src/CMakeLists.txt;0;")
