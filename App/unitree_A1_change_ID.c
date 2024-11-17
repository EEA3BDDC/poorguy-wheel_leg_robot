#include <unitree_A1_change_ID.h>
#include <usart.h>

uint32_t crc_cal_result;
static uint8_t turn_slow_Crc[34] ={0xFE,0xEE,0x02,0x00,0x0A,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x72,0x54,0x79,0x70};//

uint8_t a1_step1[34] ={0xFE,0xEE,0xBB,0x00,0x0A,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF9,0x25,0x97,0x3F};//										
										
uint8_t a1_step2[34] ={0xFE,0xEE,0xBB,0x00,0x0B,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB5,0xE8,0x2E,0xFA};//

uint8_t a1_step3[34] ={0xFE,0xEE,0xBB,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x85,0x6D,0x87};//	



uint32_t crc32_cal(uint32_t* ptr, uint32_t len){
	
	uint32_t xbit = 0;
	uint32_t data = 0;
	uint32_t CRC32 = 0xFFFFFFFF;
	const uint32_t dwPolynomial = 0x04c11db7;
	
	for (uint32_t i = 0; i < len; i++){
		xbit = 1 << 31;
		data = ptr[i];
		for (uint32_t bits = 0; bits < 32; bits++){
			if (CRC32 & 0x80000000){
				CRC32 <<= 1;
				CRC32 ^= dwPolynomial;
			}
			else
				CRC32 <<= 1;
			if (data & xbit)
				CRC32 ^= dwPolynomial;
			xbit >>= 1;
		}

	}
	return CRC32;
}

void A1_turn_slow_test(void){
	
	crc_cal_result = crc32_cal((uint32_t *) turn_slow_Crc,7);
	
	turn_slow_Crc[30] = crc_cal_result & 0xFF;
  turn_slow_Crc[31] = (crc_cal_result >> 8) & 0xFF;
  turn_slow_Crc[32] = (crc_cal_result >> 16) & 0xFF;
  turn_slow_Crc[33] = (crc_cal_result >> 24) & 0xFF;

	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)turn_slow_Crc,34);
//	HAL_GPIO_WritePin(dir1_485_GPIO_Port,dir1_485_Pin,GPIO_PIN_RESET);//RX mode	
	
	HAL_UART_Transmit_DMA(&huart6,(uint8_t *)turn_slow_Crc,34);
//	HAL_GPIO_WritePin(dir2_485_GPIO_Port,dir2_485_Pin,GPIO_PIN_RESET);	
}


void change_A1_id(void){
	
	HAL_UART_Transmit(&huart1,(uint8_t *)a1_step1,34,10);//enter close loop mode
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart1,(uint8_t *)a1_step2,34,10); // enter ,fast move motor to your want id 
	HAL_Delay(8000);
	HAL_UART_Transmit(&huart1,(uint8_t *)a1_step3,34,10); //motor slient mean id saved
	
}

