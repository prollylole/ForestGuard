# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/claudia/ForestGuard/src/ign-utils"
  "/home/claudia/ForestGuard/build_ign/test/FAKE_INSTALL-prefix/src/FAKE_INSTALL-build"
  "/home/claudia/ForestGuard/build_ign/test/FAKE_INSTALL-prefix"
  "/home/claudia/ForestGuard/build_ign/test/FAKE_INSTALL-prefix/tmp"
  "/home/claudia/ForestGuard/build_ign/test/FAKE_INSTALL-prefix/src/FAKE_INSTALL-stamp"
  "/home/claudia/ForestGuard/build_ign/test/FAKE_INSTALL-prefix/src"
  "/home/claudia/ForestGuard/build_ign/test/FAKE_INSTALL-prefix/src/FAKE_INSTALL-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/claudia/ForestGuard/build_ign/test/FAKE_INSTALL-prefix/src/FAKE_INSTALL-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/claudia/ForestGuard/build_ign/test/FAKE_INSTALL-prefix/src/FAKE_INSTALL-stamp${cfgdir}") # cfgdir has leading slash
endif()
