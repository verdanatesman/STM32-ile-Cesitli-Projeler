/*
 * LTC6802.c
 *
 *  Created on: Jun 20, 2021
 *      Author: verdan
 */


#include "LTC6802.h"

#define WRCFG 0x01 //Write Configuration Registers
#define RDCFG 0x02 // Read config
#define RDCV 0x04 // Read cells
#define STCVAD 0x10 // Start all A/D's - poll status
#define RDFLG 0x06 //Read Flags
#define RDTMP 0x08 //Read Temperatures
#define STCDC 0x60 //A/D converter and poll Status
#define STOWAD 0x20 //Start Test - poll status
#define STTMPAD 0x30// Temperature Reading - ALL
#define address 0x80

uint8_t adcv_degerler[7];
uint8_t volt_deger[18];

void yaz()
{
	uint8_t data[2];
	data[0] = address;
	data[1] = WRCFG;//deger
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET); // CS pini low yapıldı.
	HAL_SPI_Transmit(&hspi1, data, 2, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET); // CS pini low yapıldı.

	/*
Serial.println("Writing config...");
digitalWrite(10, LOW);
SPI.transfer(address);
SPI.transfer(WRCFG);
SPI.transfer(0x01);//0
SPI.transfer(0x00);//1
SPI.transfer(0x00);//2
SPI.transfer(0x00);//3
SPI.transfer(0x71);//4
SPI.transfer(0xAB);//5
digitalWrite(10, HIGH);
*/
}

void oku()
{
	uint8_t data2[2];
	data2[0] = address;
	data2[1] = RDCFG;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
	HAL_SPI_Transmit(&hspi1, data2, 2, 1000);
	HAL_SPI_Receive(&hspi1, adcv_degerler, 7, 1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);

/*
SPI.transfer(address);
SPI.transfer(RDCFG);
for(int i = 0; i < 6; i++)
{
byteTemp = SPI.transfer(RDCFG);
Serial.println(byteTemp, HEX);
}
digitalWrite(10, HIGH);

*/
}

void v_oku()
{
	uint8_t data3[1];
	data3[0] = STCVAD;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
	HAL_SPI_Transmit(&hspi1, data3, 1, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
	uint8_t data2[2];
	data2[0] = address;
	data2[1] = RDCFG;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
	HAL_SPI_Transmit(&hspi1, data2, 2, 100);
	HAL_SPI_Receive(&hspi1, volt_deger, 18, 1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);



	/*
digitalWrite(10,LOW);
SPI.transfer(STCVAD);
delay(20); // wait at least 12ms as per data sheet, p.24
digitalWrite(10,HIGH);
byte volt[18];
digitalWrite(10,LOW);
SPI.transfer(0x80);
SPI.transfer(RDCV);
for(int j = 0; j<18;j++)
{
volt[j] = SPI.transfer(RDCV);
}
digitalWrite(10,HIGH);
Serial.println(((volt[0] & 0xFF) | (volt[1] & 0x0F) << 8)*1.5*0.001);
Serial.println(((volt[1] & 0xF0) >> 4 | (volt[2] & 0xFF) << 4)*1.5*0.001);
Serial.println(((volt[3] & 0xFF) | (volt[4] & 0x0F) << 8)*1.5*0.001);
Serial.println(((volt[4] & 0xF0) >> 4 | (volt[5] & 0xFF) << 4)*1.5*0.001);
Serial.println("--------------------");
*/
}
