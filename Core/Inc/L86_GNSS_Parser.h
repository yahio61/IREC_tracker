/*
 * usr_gnss_l86_parser.h
 *
 *  Created on: Mar 16, 2024
 *      Author: numan
 */

//pmtk commands
//HAL_UART_Transmit(&GPS_UART_HNDLR, "$PMTK251,57600*2C\r\n", 19, 100);				// 57600 bps
//HAL_UART_Transmit(&GPS_UART_HNDLR, "$PMTK251,38400*27\r\n", 19, 100);				// 38400 bps
//HAL_UART_Transmit(&GPS_UART_HNDLR, "$PMTK251,9600*17\r\n", 18, 100);				// 9600 bps

#ifndef L86_GNSS_PARSER_H_
#define L86_GNSS_PARSER_H_

#define VIEW_GPS()		while(1){	\
		 	 	 	 	 if(__HAL_UART_GET_FLAG(&GPS_UART_HNDLR, UART_FLAG_RXNE) == SET) {	\
		 	 	 	 		 uint8_t dat[1];										\
		 	 	 	 	 	 HAL_UART_Receive(&GPS_UART_HNDLR, (uint8_t *) dat, 1, 100);	\
		 	 	 	 	 	 HAL_UART_Transmit(&TTL_UART_HNDLR, (uint8_t *) dat, 1, 100);	\
		 	 	 	 	 	 }														\
		 	 	 	 	 }

#include "main.h"
#include "usr_gnss_general.h"

typedef bool _f; // flag type

typedef struct S_GPS_L86_DATA_TAG
{
    float lat;
    float lon;
    float speedKN;
    float timeDateBuf;
    float fixedTime;
    float fixedLatBaseFormat;
    float fixedLonBaseFormat;
    int fixQualityID;
    int satInUse;
    float hdop;
    float altitudeInMeter;
    float WGS84;
    char east_west;
}S_GPS_L86_DATA;


void UsrGpsL86Init(UART_HandleTypeDef *huart);
/*
 * L86 initial
 * Starts the dma
 *
 * */
void Usr_GpsL86GetValues(S_GPS_L86_DATA *gpsData_);
/*
 * data set for L86
 *
    gpsData.lat;
   gpsData.lon;
   gpsData.hdop;
   gpsData.speedKN;
   gpsData.satInUse;
   gpsData.timeDateBuf;
   gpsData.fixQualityID;
   gpsData.altitudeInMeter;

 * 2nd structure for the usr
 * */

#endif /* USR_GNSS_L86_PARSER_H_ */
