
#ifndef SPI_BMP_H_
#define SPI_BMP_H_

#include <stm32f10x_conf.h>
#include <stm32f1xx_it.h>

#define BMP280Write     0xEC
#define BMP280Read      0xED

//void Conf_SPI1_Master(void);
void BMP280_SPI2Master_Config(void);
void BMP280_get_calib_values(void);
void BMP280_calc_values(void);
void BMP280_Init(void);
void BMP280_Write(uint8_t endereco, uint8_t dado);
void BMP280_Read(uint8_t endereco, uint8_t *buffer, uint8_t tam);
uint8_t BMP280_Transfer(uint8_t outByte);

#endif /* SPI_BMP_H_ */
