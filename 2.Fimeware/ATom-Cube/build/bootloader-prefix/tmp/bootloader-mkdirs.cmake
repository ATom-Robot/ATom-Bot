# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/bootloader/subproject"
  "D:/workspace_rb/ATom-Bot/2.Fimeware/ATom-Cube/build/bootloader"
  "D:/workspace_rb/ATom-Bot/2.Fimeware/ATom-Cube/build/bootloader-prefix"
  "D:/workspace_rb/ATom-Bot/2.Fimeware/ATom-Cube/build/bootloader-prefix/tmp"
  "D:/workspace_rb/ATom-Bot/2.Fimeware/ATom-Cube/build/bootloader-prefix/src/bootloader-stamp"
  "D:/workspace_rb/ATom-Bot/2.Fimeware/ATom-Cube/build/bootloader-prefix/src"
  "D:/workspace_rb/ATom-Bot/2.Fimeware/ATom-Cube/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/workspace_rb/ATom-Bot/2.Fimeware/ATom-Cube/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
