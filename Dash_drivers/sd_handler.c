// /*
//  * sd_handler.c
//  *
//  * Created: 27-Feb-16 6:50:14 PM
//  *  Author: Walid
//  */ 
// //Create new log data file everytime the pc is restartet?
// #include "sd_handler.h"
// #include <string.h>
// //#include "sam.h"
// //#include "conf_access.h"
// 
// 
// 
// FRESULT fat_createNewFile(char *filename,FATFS *fs, FIL *fileobject){
// 	//char *filenamecreated = filename;
// 	//strcat(filenamecreated, ".txt");
// 	//sprintf(filenamecreated,LUN_ID_SD_MMC_0_MEM + '0'+ filename);
// 	//strcpy(filenamecreated,filename + ".txt");
// 	memset(fs, 0, sizeof(FATFS));
// 	if(f_mount(LUN_ID_SD_MMC_0_MEM, fs) == FR_INVALID_DRIVE)
// 		return FR_INVALID_DRIVE;
// 	//filename[0] = LUN_ID_SD_MMC_0_MEM + '0';
// 	FRESULT res = f_open(fileobject,(char const *)filename, FA_CREATE_NEW | FA_WRITE);
// 	if(res != FR_OK)
// 		return res;
// 	f_puts("start: ", fileobject);
// 	f_close(fileobject);
// 	
// 	return FR_OK;		
// }
// 
// FRESULT fat_writeToFileS(char *filename, FATFS *fs, FIL *fileobject, char *string ){
// 	//check if file exists
// 	FILINFO fno;
// 	f_mount(LUN_ID_SD_MMC_0_MEM, fs);
// 	FRESULT res = f_stat(filename,&fno);
// 
// 	switch(res) {
// 	
// 		case FR_OK:
// 			f_open(fileobject,filename,FA_WRITE | FA_OPEN_EXISTING);
// 			f_lseek(fileobject,f_size(fileobject));
// 			//f_printf(fileobject, string);
// 			f_puts(string,fileobject);
// 			//f_printf(fileobject, "hello from printf");
// 			f_close(fileobject);
// 			return res;
// 		case FR_NO_FILE:
// 			res == fat_createNewFile(filename,fs,fileobject);
// 			if(res != FR_OK)
// 				return res;
// 			f_open(fileobject,filename,FA_WRITE | FA_OPEN_EXISTING);
// 			//f_printf(fileobject,string);
// 			f_lseek(fileobject,sizeof(fileobject));
// 			f_printf(fileobject,string);
// 			f_close(fileobject);
// 			return res;
// 		default:
// 			return res;
// 	}
// 
// 	return res;
// 	
// }
// 
// FRESULT fat_writeToFileB(char *filename, FATFS *fs, FIL *fileobject,uint8_t *byte, uint8_t size){
// 	f_mount(LUN_ID_SD_MMC_0_MEM,fs);
// 	f_open(fileobject,(char const *)filename, FA_OPEN_EXISTING | FA_WRITE);
// 	f_lseek(fileobject,sizeof(fileobject));
// 	f_putc(80,fileobject);
// 	f_puts("  ",fileobject);
// 
// 	f_close(fileobject);
// 	
// 	
// }
// 
// /*FRESULT fat_writeCanData(uint32_t index){
// 	FATFS fs;
// 	FIL fileobject;
// 	struct Can_message_t rx_msg;
// 	can_getMessage(MCAN1, &rx_msg, index);
// 	fat_writeToFile("0:canData.txt",&fs,&fileobject,rx_msg.data);
// 	
// }*/
// 
// FRESULT fat_deleteFile(char *filename){
// 	f_unlink(filename);
// }
// //write to current open file
// //FRESULT fat_writeTCOF(){			
// //}
// 
