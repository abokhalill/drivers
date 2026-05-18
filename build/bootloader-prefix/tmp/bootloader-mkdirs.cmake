# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/leena/drivers/esp-idf/components/bootloader/subproject"
  "/home/leena/drivers/esp32-suspension/build/bootloader"
  "/home/leena/drivers/esp32-suspension/build/bootloader-prefix"
  "/home/leena/drivers/esp32-suspension/build/bootloader-prefix/tmp"
  "/home/leena/drivers/esp32-suspension/build/bootloader-prefix/src/bootloader-stamp"
  "/home/leena/drivers/esp32-suspension/build/bootloader-prefix/src"
  "/home/leena/drivers/esp32-suspension/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/leena/drivers/esp32-suspension/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/leena/drivers/esp32-suspension/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
