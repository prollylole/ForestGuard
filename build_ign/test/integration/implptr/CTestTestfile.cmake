# CMake generated Testfile for 
# Source directory: /home/claudia/ForestGuard/src/ign-utils/test/integration/implptr
# Build directory: /home/claudia/ForestGuard/build_ign/test/integration/implptr
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(INTEGRATION_ImplPtr_TEST "/home/claudia/ForestGuard/build_ign/bin/INTEGRATION_ImplPtr_TEST" "--gtest_output=xml:/home/claudia/ForestGuard/build_ign/test_results/INTEGRATION_ImplPtr_TEST.xml")
set_tests_properties(INTEGRATION_ImplPtr_TEST PROPERTIES  TIMEOUT "240" _BACKTRACE_TRIPLES "/usr/share/cmake/ignition-cmake2/cmake2/IgnUtils.cmake;1725;add_test;/home/claudia/ForestGuard/src/ign-utils/test/integration/implptr/CMakeLists.txt;14;ign_build_tests;/home/claudia/ForestGuard/src/ign-utils/test/integration/implptr/CMakeLists.txt;0;")
add_test(check_INTEGRATION_ImplPtr_TEST "/usr/bin/python3.10" "/usr/share/ignition/ignition-cmake2/tools/check_test_ran.py" "/home/claudia/ForestGuard/build_ign/test_results/INTEGRATION_ImplPtr_TEST.xml")
set_tests_properties(check_INTEGRATION_ImplPtr_TEST PROPERTIES  _BACKTRACE_TRIPLES "/usr/share/cmake/ignition-cmake2/cmake2/IgnUtils.cmake;1741;add_test;/home/claudia/ForestGuard/src/ign-utils/test/integration/implptr/CMakeLists.txt;14;ign_build_tests;/home/claudia/ForestGuard/src/ign-utils/test/integration/implptr/CMakeLists.txt;0;")
