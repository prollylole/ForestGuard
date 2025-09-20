# CMake generated Testfile for 
# Source directory: /home/claudia/ForestGuard/src/ign-utils/test/integration
# Build directory: /home/claudia/ForestGuard/build_ign/test/integration
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(INTEGRATION_versioned_symbols "bash" "/home/claudia/ForestGuard/build_ign/test/integration/all_symbols_have_version.bash" "/home/claudia/ForestGuard/build_ign/lib/libignition-utils1.so.1.5.1")
set_tests_properties(INTEGRATION_versioned_symbols PROPERTIES  _BACKTRACE_TRIPLES "/home/claudia/ForestGuard/src/ign-utils/test/integration/CMakeLists.txt;8;add_test;/home/claudia/ForestGuard/src/ign-utils/test/integration/CMakeLists.txt;0;")
add_test(INTEGRATION_deprecated_TEST "/home/claudia/ForestGuard/build_ign/bin/INTEGRATION_deprecated_TEST" "--gtest_output=xml:/home/claudia/ForestGuard/build_ign/test_results/INTEGRATION_deprecated_TEST.xml")
set_tests_properties(INTEGRATION_deprecated_TEST PROPERTIES  TIMEOUT "240" _BACKTRACE_TRIPLES "/usr/share/cmake/ignition-cmake2/cmake2/IgnUtils.cmake;1725;add_test;/home/claudia/ForestGuard/src/ign-utils/test/integration/CMakeLists.txt;12;ign_build_tests;/home/claudia/ForestGuard/src/ign-utils/test/integration/CMakeLists.txt;0;")
add_test(check_INTEGRATION_deprecated_TEST "/usr/bin/python3.10" "/usr/share/ignition/ignition-cmake2/tools/check_test_ran.py" "/home/claudia/ForestGuard/build_ign/test_results/INTEGRATION_deprecated_TEST.xml")
set_tests_properties(check_INTEGRATION_deprecated_TEST PROPERTIES  _BACKTRACE_TRIPLES "/usr/share/cmake/ignition-cmake2/cmake2/IgnUtils.cmake;1741;add_test;/home/claudia/ForestGuard/src/ign-utils/test/integration/CMakeLists.txt;12;ign_build_tests;/home/claudia/ForestGuard/src/ign-utils/test/integration/CMakeLists.txt;0;")
subdirs("implptr")
