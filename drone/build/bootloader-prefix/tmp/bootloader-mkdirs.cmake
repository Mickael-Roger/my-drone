# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/nix/store/id5c9bf5kk251aa76404lhh7zf0b82dx-esp-idf-v5.4/components/bootloader/subproject")
  file(MAKE_DIRECTORY "/nix/store/id5c9bf5kk251aa76404lhh7zf0b82dx-esp-idf-v5.4/components/bootloader/subproject")
endif()
file(MAKE_DIRECTORY
  "/home/mickael/Documents/Maker/my-drone/drone/build/bootloader"
  "/home/mickael/Documents/Maker/my-drone/drone/build/bootloader-prefix"
  "/home/mickael/Documents/Maker/my-drone/drone/build/bootloader-prefix/tmp"
  "/home/mickael/Documents/Maker/my-drone/drone/build/bootloader-prefix/src/bootloader-stamp"
  "/home/mickael/Documents/Maker/my-drone/drone/build/bootloader-prefix/src"
  "/home/mickael/Documents/Maker/my-drone/drone/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/mickael/Documents/Maker/my-drone/drone/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/mickael/Documents/Maker/my-drone/drone/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
