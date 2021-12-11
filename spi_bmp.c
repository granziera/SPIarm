#include "spi_bmp.h"
#include <stdio.h>
#include <math.h>

signed long temperature_raw, pressure_raw;
unsigned short dig_T1, dig_P1;
signed short dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
float temperature, pressure, altitude;


uint8_t BMP280_Transfer(uint8_t comando)
{
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, comando);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	return SPI_I2S_ReceiveData(SPI2);
}

void BMP280_Write(uint8_t endereco, uint8_t dado)
{
	SPI_SSOutputCmd(SPI2, ENABLE);
	BMP280_Transfer(endereco);
	BMP280_Transfer(dado);
  	SPI_SSOutputCmd(SPI2, DISABLE);
}

void BMP280_Read(uint8_t endereco, uint8_t *buffer, uint8_t tam)
{
	SPI_SSOutputCmd(SPI2, ENABLE);
	BMP280_Transfer(endereco);
	for (int i = 0; i < tam; i++)
	{
		*buffer = BMP280_Transfer(0x00);
		buffer++;
		//TIM3_Delay(1);
	}
	SPI_SSOutputCmd(SPI2, DISABLE);
}

void BMP280_Init(void)
{
	BMP280_Write(0x75, 0x90);
	BMP280_Write(0x74, 0x57);
	BMP280_get_calib_values();
}

void BMP280_get_calib_values(void)
{
	uint8_t rx_buff[24], starting_address=0x88;

	BMP280_Read(starting_address, rx_buff, 24);

	dig_T1 = (rx_buff[0]) + (rx_buff[1]<<8);
	dig_T2 = (rx_buff[2]) + (rx_buff[3]<<8);
	dig_T3 = (rx_buff[4]) + (rx_buff[5]<<8);
	dig_P1 = (rx_buff[6]) + (rx_buff[7]<<8);
	dig_P2 = (rx_buff[8]) + (rx_buff[9]<<8);
	dig_P3 = (rx_buff[10]) + (rx_buff[11]<<8);
	dig_P4 = (rx_buff[12]) + (rx_buff[13]<<8);
	dig_P5 = (rx_buff[14]) + (rx_buff[15]<<8);
	dig_P6 = (rx_buff[16]) + (rx_buff[17]<<8);
	dig_P7 = (rx_buff[18]) + (rx_buff[19]<<8);
	dig_P8 = (rx_buff[20]) + (rx_buff[21]<<8);
	dig_P9 = (rx_buff[22]) + (rx_buff[23]<<8);
}

void BMP280_calc_values(void)
{
	uint8_t status, rx_buff[6], starting_address=0xF7;
	do
	{
		BMP280_Read(0xF3, &status, 1);
	} while(((status&0b00001000)==8)||((status&0b00000001)==1));

	GPIO_WriteBit(GPIOA, GPIO_Pin_3, 0);
	BMP280_Transfer(starting_address);

	for(int i = 0; i < 6; i++)
	{
		rx_buff[i] = BMP280_Transfer(0x00);
	}

	GPIO_WriteBit(GPIOA, GPIO_Pin_3, 1);

	volatile uint32_t temp[3];
	temp[2] = rx_buff[3];
	temp[1] = rx_buff[4];
	temp[0] = rx_buff[5];
	temperature_raw = (temp[2]<<12) + (temp[1]<<4) + (temp[0]>>4);

	temp[2] = rx_buff[0];
	temp[1] = rx_buff[1];
	temp[0] = rx_buff[2];
	pressure_raw = (temp[2]<<12) + (temp[1]<<4) + (temp[0]>>4);

	double var1, var2;
	var1 = (((double)temperature_raw)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
	var2 = ((((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0)*(((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0))*((double)dig_T3);
	double t_fine = (int32_t)(var1+var2);
	volatile float T = (var1+var2)/5120.0;
	var1=((double)t_fine/2.0)-64000.0;
	var2=var1*var1*((double)dig_P6)/32768.0;
	var2=var2+var1*((double)dig_P5)*2.0;
	var2=(var2/4.0)+(((double)dig_P4)*65536.0);
	var1=(((double)dig_P3)*var1*var1/524288.0+((double)dig_P2)*var1)/524288.0;
	var1=(1.0+var1/32768.0)*((double)dig_P1);

	volatile double p=1048576.0-(double)pressure_raw;
	p=(p-(var2/4096.0))*6250.0/var1;
	var1=((double)dig_P9)*p*p/2147483648.0;
	var2=p*((double)dig_P8)/32768.0;
	p=p+(var1+var2+((double)dig_P7))/16.0;

	temperature=T;
	pressure=p;
	//altitude=44330.0f*(1-powf(pressure/101325.0f,1.0f/5.255f));
	//altitude=((powf(101325.0/pressure, 1/5.257f)-1)*(temperature+273.15f))/0.0065f;
}


void BMP280_SPI2Master_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef my_gpio;
	my_gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	my_gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
	my_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &my_gpio);
	my_gpio.GPIO_Mode = GPIO_Mode_IPU;
	my_gpio.GPIO_Pin = GPIO_Pin_14;
	my_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &my_gpio);

	SPI_InitTypeDef my_spi;

	my_spi.SPI_Mode = SPI_Mode_Master;
	my_spi.SPI_DataSize = SPI_DataSize_8b;
	my_spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	my_spi.SPI_FirstBit = SPI_FirstBit_MSB;
	my_spi.SPI_CPOL = SPI_CPOL_High;
	my_spi.SPI_CPHA = SPI_CPHA_2Edge;
	my_spi.SPI_CRCPolynomial = 0x0000;
	my_spi.SPI_NSS = SPI_NSS_Soft;
	my_spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;

	SPI_Init (SPI2, &my_spi);
	SPI_Cmd (SPI2, ENABLE);

}
