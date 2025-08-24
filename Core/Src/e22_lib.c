/*
 * e22_lib.c
 *
 *  Created on: Dec 20, 2024
 *      Author: yahya
 */
#include "e22_lib.h"

/**
  * @brief  Initializes the lora module by configuration struct.
  * @param  lora_conf_struct: pointer to the lora configuration struct.
  * @param  huart: pointer to the uart handler typedef.
  * @retval None
  */
void e22_init(e22_conf_struct_t *lora_conf_struct, UART_HandleTypeDef* huart)
{
	uint8_t data_packet[9] = {0};
	data_packet[0] = 0xC0;	//Set register command.
	data_packet[1] = 0x03;	//Starting from byte 0x03
	data_packet[2] = 0x06;	//6 bytes will be configured.
	data_packet[3] = lora_conf_struct->air_rate | lora_conf_struct->parity_bit << 3 | lora_conf_struct->baud_rate << 5;																									//Wireless air data rate（bps）, Serial parity bit, UART Serial port rate（bps).
	data_packet[4] = lora_conf_struct->power | lora_conf_struct->rssi_noise << 5 | lora_conf_struct->packet_size << 6;																									//Transmitting power, RSSI anbient noise enable, Sub packet settings.
	data_packet[5] = lora_conf_struct->channel;																																											//channel 0-83 (410.125 + CH *1M)
	data_packet[6] = lora_conf_struct->wor_cycle | lora_conf_struct->wor << 3 | lora_conf_struct->lbt << 4 | lora_conf_struct->repeater_func << 5 | lora_conf_struct->mode << 6 | lora_conf_struct->rssi_enable << 7;	//WOR cycle time, WOR transceiver control, LBT enable, Repeater function, Transmission mode, Enable RSSI.
	data_packet[7] = (uint8_t)(lora_conf_struct->key >> 8);																																								//high byte of key
	data_packet[8] = (uint8_t)(lora_conf_struct->key);
																																								//low byte of key
	//For config mode M0 -> 0    M1 -> 1
	HAL_GPIO_WritePin(lora_conf_struct->pins.m0_pin_port, lora_conf_struct->pins.m0_pin, RESET);
	HAL_GPIO_WritePin(lora_conf_struct->pins.m1_pin_port, lora_conf_struct->pins.m1_pin, SET);

	HAL_Delay(100);

	//UART transmits the configuration datas.
	HAL_UART_Transmit(huart, data_packet, 9, 50);
	HAL_Delay(50);
}

/**
  * @brief  Transmits the data.
  * @param  lora_conf_struct: Pointer to the lora configuration struct.
  * @param  data: Pointer to the data string.
  * @param  huart: Pointer to the uart handler typedef.
  * @retval None
  */
void e22_transmit(e22_conf_struct_t *lora_conf_struct, uint8_t* data, UART_HandleTypeDef* huart, uint16_t length)
{
	//For normal working mode M0 -> 0    M1 -> 0
	HAL_GPIO_WritePin(lora_conf_struct->pins.m0_pin_port, lora_conf_struct->pins.m0_pin, RESET);
	HAL_GPIO_WritePin(lora_conf_struct->pins.m1_pin_port, lora_conf_struct->pins.m1_pin, RESET);
	HAL_Delay(20);

	HAL_UART_Transmit(huart, data, length, 100);
}

/**
  * @brief  Makes the module asleep.
  * @param  lora_conf_struct: Pointer to the lora configuration struct.
  * @retval None
  */
void e22_sleep(e22_conf_struct_t *lora_conf_struct)
{
	//For sleep mode M0 -> 1    M1 -> 1
	HAL_GPIO_WritePin(lora_conf_struct->pins.m0_pin_port, lora_conf_struct->pins.m0_pin, SET);
	HAL_GPIO_WritePin(lora_conf_struct->pins.m1_pin_port, lora_conf_struct->pins.m1_pin, SET);
	HAL_Delay(20);
}

