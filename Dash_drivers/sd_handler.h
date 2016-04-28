// /*
//  * sd_handler.h
//  *
//  * Created: 27-Feb-16 6:33:19 PM
//  *  Author: Walid
//  */ 
// 
// 
// #ifndef SD_HANDLER_H_
// #define SD_HANDLER_H_
// 
// #include <stdint.h>
// #include "sd_mmc/fat/ff.h"
// #include "../ctrl_access.h"
// 
// typedef union {
// 	uint32_t u32;
// 	uint16_t u16;
// 	uint8_t u8;
// }Datatype;
// 
// 
// 
// //Should open a file, write to the file then close it.
// FRESULT fat_createNewFile(char *filename,FATFS *fs, FIL *fileobject);
// FRESULT fat_writeToFileS(char *filename, FATFS *fs, FIL *fileobject, char *string);
// FRESULT fat_writeToFileB(char *filename, FATFS *fs, FIL *fileobject,uint8_t *byte,uint8_t size);
// //FRESULT fat_writeCanData(uint32_t index);
// 
// 
// #endif /* SD_HANDLER_H_ */