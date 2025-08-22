#ifndef _FILE_SYSTEM_H
#define  _FILE_SYSTEM_H
#include <Arduino.h>
#include "maingrobal.h"
// class PrintExt : public Print{
//     PrintExt(){};
//     size_t write(u_int8_t c){
//         Serial.write(c);
//     }
// };
typedef struct {
  time_t logTime;
  u_int16_t status;
  u_int16_t fault;
} upslog_t;

class LittleFileSystem
{
    public:
    LittleFileSystem();
    void littleFsInitFast(int bformat);
    void littleFsInit(int bformat);
    void setOutputStream(Print* stream);
    int rm(String fileName);
    void listDir(const char *path, char *match);
    int format();
    void cat(String finename);
    int fnmatch(const char *pattern, const char *string, int flags);
    const char* rangematch(const char *pattern, char test, int flags); 
    void df();
    int writeLog(time_t logtime,u_int16_t status,u_int16_t fault);
    int writeMeasuredValue(_cell_value_iv value);
    int writeCellDataLog();
    int readCellDataLog(bool isBoot);
    int readMeasuredValue();
    int writeLogString(String log);
    void printCellLogData(cell_logData_t *cell_logData);
    void fillCellLogData(cell_logData_t *cell_logData);

    private:
    Print* outputStream;
};
#endif