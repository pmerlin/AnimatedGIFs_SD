#ifndef FILENAME_FUNCTIONS_H
#define FILENAME_FUNCTIONS_H


//#define FILESYSTEM LittleFS
#define FILESYSTEM SPIFFS

int enumerateGIFFiles(const char *directoryName, bool displayFilenames);
void getGIFFilenameByIndex(const char *directoryName, int index, char *pnBuffer);
int openGifFilenameByName(const char *Name);
int openGifFilenameByIndex(const char *directoryName, int index);
int initSdCard(int chipSelectPin);

bool fileSeekCallback(unsigned long position);
unsigned long filePositionCallback(void);
int fileReadCallback(void);
int fileReadBlockCallback(void * buffer, int numberOfBytes);

#endif
