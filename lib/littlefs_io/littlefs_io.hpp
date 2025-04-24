/* EcoDrone: Autonomous Environmental Monitoring
 * Header file for writing into LittleFS file system on ESP32
 * Author: Brandon Lee, brandon.kf.lee@gmail.com
 * Code derived from Arduino ESP32 LittleFS example (https://github.com/espressif/arduino-esp32/blob/master/libraries/LittleFS/examples/LITTLEFS_test/LITTLEFS_test.ino)
 */ 

#include <Arduino.h>
#include "FS.h"
#include <LittleFS.h>

/* You only need to format LittleFS the first time you run a
   test or else use the LITTLEFS plugin to create a partition
   https://github.com/lorol/arduino-esp32littlefs-plugin

   If you test two partitions, you need to use a custom
   partition.csv file, see in the sketch folder */

/* TODO: create a class? */

#define FORMAT_LITTLEFS_IF_FAILED true

void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
String readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);

// SPIFFS-like write and delete file, better use #define CONFIG_LITTLEFS_SPIFFS_COMPAT 1
void writeFile2(fs::FS &fs, const char *path, const char *message);
void deleteFile2(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);