#ifndef _FILE_SYSTEM_H
#define  _FILE_SYSTEM_H
#include <Arduino.h>
// class PrintExt : public Print{
//     PrintExt(){};
//     size_t write(u_int8_t c){
//         Serial.write(c);
//     }
// };
class LittleFileSystem
{
    public:
    LittleFileSystem();
    void littleFsInitFast(int bformat);
    void littleFsInit(int bformat);
    void setOutputStream(Print* stream);
    void listDir(const char *path, char *match);
    int format();
    void cat(String finename);
    int fnmatch(const char *pattern, const char *string, int flags);
    const char* rangematch(const char *pattern, char test, int flags); 
    void df();
    private:
    Print* outputStream;
};
#endif