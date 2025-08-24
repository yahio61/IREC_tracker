/*
 * dataPacking.h
 *
 *  Created on: 29 Tem 2024
 *      Author: yahya
 */

#ifndef INC_DATAPACKING_H_
#define INC_DATAPACKING_H_

#include <stdint.h>
#include "main.h"
#include "L86_GNSS_Parser.h"
#include "bme280.h"

struct DataStruct
{
	uint8_t basla;
	uint8_t zaman; 			//ilk 6 bit dakika sonraki 2 bit, saniyenin 5. ve 4. bitleri oluyor
	uint8_t durum;			//ilk 4 bit saniyenin 3. 4. 1. 0. bitleri. son 4 bit ise durum.
/*
 * 	(0001 -> roket hazır   	0010 -> ucus basladi
 * 	(0011 -> motor bitti	0100 -> P1-1 P2-0
 * 	(0101 -> P1-1 P2-1		0110 -> P1-0 P2-1
 * 	(0111 -> ucus bitti		1000 ->
*/
	int8_t sicaklik;		//floata çevrilip 5'e bölünmeli, 0.2 derece hassasiyetinde veri gelmekte
	uint16_t voltaj;		//floata çevrilip 100'bölünmeli. 100mV hassasiyette veri gelmekte.
	uint16_t akim;
	float 	yukseklik_p;
	float 	yukseklik_gps;
	float	lat;
	float	lon;
	float	gyroX;
	float	gyroY;
	float	gyroZ;
	float	accX;
	float	accY;
	float	accZ;
	float	aci;
	uint8_t nem;
	uint8_t pitch;			//pitch verisinin 7-0 bitleri
	uint8_t roll;			//roll verisinin 7-0 bitleri
	uint8_t yaw;			//yaw verisinin 7-0 bitleri
	int16_t hiz;			//hiz değeri 10 ile çarpılıp m/s cinsinden göderilir. Alındığında 10'a bölünüp float yapılmalı.
	int16_t maxAltitude;	//Max altitude that rocket launched;
	uint8_t uyduSayisi;		//Son 5 bit uydu sayısını vermektedir.
	/*
	 * 2. bit pithc verisinin 9. biti
	 * 1. bit roll verisinin 9. biti
	 * 0. bit yaw verisinin 9. biti
	 */
	uint8_t checkSum;
	uint8_t	CR;
	uint8_t LF;
	//toplam 64 byte padding yok.
};

typedef union DataPack
{
	uint8_t arr[64];
	struct DataStruct dataYapi;
}data_pack_t;

typedef union half_word_to_bytes {
    uint16_t 	half_word;
    uint8_t 	byte[2];
} half_word_to_bytes_t;

uint8_t* packDatas(S_GPS_L86_DATA *gps, uint16_t voltage);
void printDatas();
#endif /* INC_DATAPACKING_H_ */
