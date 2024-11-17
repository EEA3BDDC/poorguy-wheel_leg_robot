#include "main.h"
#include "motor_msg.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "unitreeA1_cmd.h"

motor_send_t cmd_left; // 左腿电机数据体
motor_send_t cmd_right; // 右腿电机数据体

motor_recv_t Data_left;      // 右腿电机接收数据体
motor_recv_t id01_left_data; // 右腿01号电机接收数据体
motor_recv_t id02_left_data; // 右腿02号电机接收数据体

motor_recv_t Data_right;      // 右腿电机接收数据体
motor_recv_t id01_right_data; // 右腿01号电机接收数据体
motor_recv_t id02_right_data; // 右腿02号电机接收数据体

extern uint8_t Left_leg_RevData[78];
extern uint8_t Right_leg_RevData[78];

	uint8_t A1cmd_left[34];
	uint8_t A1cmd_right[34];


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){	
		
		Data_left.motor_recv_data.head.motorID = Left_leg_RevData[2];
		Data_left.motor_recv_data.Mdata.MError = Left_leg_RevData[7];
		Data_left.motor_recv_data.Mdata.T = Left_leg_RevData[12] << 8 | Left_leg_RevData[13];
		Data_left.motor_recv_data.Mdata.Pos2 = Left_leg_RevData[30] << 24 | Left_leg_RevData[31] << 16 | Left_leg_RevData[32] << 8 | Left_leg_RevData[33];

		Data_left.motor_id = Data_left.motor_recv_data.head.motorID;
		Data_left.MError = Data_left.motor_recv_data.Mdata.MError;
		Data_left.T = Data_left.motor_recv_data.Mdata.T / 256;
		Data_left.Pos = (int)((Data_left.motor_recv_data.Mdata.Pos2 / 16384.0f) * 6.2832f);
		
		if (Data_left.motor_id == 0x01)
		{
				id01_left_data.motor_id = Data_left.motor_id;
				id01_left_data.MError = Data_left.MError;
				id01_left_data.T = Data_left.T;
				id01_left_data.Pos = Data_left.Pos;
		}
		if (Data_left.motor_id == 0x02)
		{
				id02_left_data.motor_id = Data_left.motor_id;
				id02_left_data.MError = Data_left.MError;
				id02_left_data.T = Data_left.T;
				id02_left_data.Pos = Data_left.Pos;
		}
		
		HAL_GPIO_WritePin(dir1_485_GPIO_Port,dir1_485_Pin,GPIO_PIN_SET);	//set 485chip as TX mode
		HAL_UART_Receive_DMA(&huart1,(uint8_t *)Left_leg_RevData,78); // ????DMA?? 	
	}
	
	if(huart->Instance == USART6){
		
		Data_right.motor_recv_data.head.motorID = Right_leg_RevData[2];
		Data_right.motor_recv_data.Mdata.MError = Right_leg_RevData[7];
		Data_right.motor_recv_data.Mdata.T = Right_leg_RevData[12] << 8 | Right_leg_RevData[13];
		Data_right.motor_recv_data.Mdata.Pos2 = Right_leg_RevData[30] << 24 | Right_leg_RevData[31] << 16 | Right_leg_RevData[32] << 8 | Right_leg_RevData[33];

		Data_right.motor_id = Data_right.motor_recv_data.head.motorID;
		Data_right.MError = Data_right.motor_recv_data.Mdata.MError;
		Data_right.T = Data_right.motor_recv_data.Mdata.T / 256;
		Data_right.Pos = (int)((Data_right.motor_recv_data.Mdata.Pos2 / 16384.0f) * 6.2832f);

		if (Data_right.motor_id == 0x01)
		{
				id01_right_data.motor_id = Data_right.motor_id;
				id01_right_data.MError = Data_right.MError;
				id01_right_data.T = Data_right.T;
				id01_right_data.Pos = Data_right.Pos;
		}

		if (Data_right.motor_id == 0x02)
		{
				id02_right_data.motor_id = Data_right.motor_id;
				id02_right_data.MError = Data_right.MError;
				id02_right_data.T = Data_right.T;
				id02_right_data.Pos = Data_right.Pos;
		}
		
		HAL_GPIO_WritePin(dir2_485_GPIO_Port,dir2_485_Pin,GPIO_PIN_SET);	//set 485chip as TX mode		
		HAL_UART_Receive_DMA(&huart6,(uint8_t *)Right_leg_RevData,78); // ????DMA?? 		
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	
	if(huart->Instance == USART1){
		HAL_GPIO_WritePin(dir1_485_GPIO_Port,dir1_485_Pin,GPIO_PIN_RESET);//set 485chip as RX mode
	}
	if(huart->Instance == USART6){
		HAL_GPIO_WritePin(dir2_485_GPIO_Port,dir2_485_Pin,GPIO_PIN_RESET);//set 485chip as RX mode
	}
}

// CRC校验位的代码
uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    uint32_t bits;
    uint32_t i;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
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

void modfiy_cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW)
{

    send->hex_len = 34;  
    send->mode = 10;     //closed loop
		send->id = id;
    send->K_P = KP;
    send->K_W = KW;
    send->Pos = 6.2832*9.1*2*Pos;
    send->W = 0;
    send->T = 0.0;
}

void unitreeA1_rxtx(UART_HandleTypeDef *huart)
{

	
    if (huart == &huart1)
    {
        /*—————————————————————————————————————————左腿代码范围————————————————————————————————————————————————*/
        // 此处为左腿电机结构体//
        cmd_left.motor_send_data.head.start[0] = 0xFE;
        cmd_left.motor_send_data.head.start[1] = 0xEE;
        cmd_left.motor_send_data.head.motorID = cmd_left.id;
        cmd_left.motor_send_data.head.reserved = 0x00;

        cmd_left.motor_send_data.Mdata.mode = cmd_left.mode;
        cmd_left.motor_send_data.Mdata.ModifyBit = 0xFF;
        cmd_left.motor_send_data.Mdata.ReadBit = 0x00;
        cmd_left.motor_send_data.Mdata.reserved = 0x00;
        cmd_left.motor_send_data.Mdata.Modify.F = 0;
        cmd_left.motor_send_data.Mdata.T = cmd_left.T * 256;
        cmd_left.motor_send_data.Mdata.W = cmd_left.W * 128;
        cmd_left.motor_send_data.Mdata.Pos = (int)((cmd_left.Pos / 6.2832f) * 16384.0f);
        cmd_left.motor_send_data.Mdata.K_P = cmd_left.K_P * 2048;
        cmd_left.motor_send_data.Mdata.K_W = cmd_left.K_W * 1024;
        cmd_left.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        cmd_left.motor_send_data.Mdata.LowHzMotorCmdByte = 0;
        cmd_left.motor_send_data.Mdata.Res[0] = cmd_left.Res;

        cmd_left.motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)(&cmd_left.motor_send_data), 7);

        memcpy(A1cmd_left, &cmd_left.motor_send_data, 34);

        HAL_UART_Transmit_DMA(&huart1, A1cmd_left, 34);
    }

    if (huart == &huart6)
    {
        /*—————————————————————————————————————————右腿代码范围————————————————————————————————————————————————————————*/
        // 此处为右腿一号电机结构体//
        cmd_right.motor_send_data.head.start[0] = 0xFE;
        cmd_right.motor_send_data.head.start[1] = 0xEE;
        cmd_right.motor_send_data.head.motorID = cmd_right.id;
        cmd_right.motor_send_data.head.reserved = 0x00;

        cmd_right.motor_send_data.Mdata.mode = cmd_right.mode;
        cmd_right.motor_send_data.Mdata.ModifyBit = 0xFF;
        cmd_right.motor_send_data.Mdata.ReadBit = 0x00;
        cmd_right.motor_send_data.Mdata.reserved = 0x00;
        cmd_right.motor_send_data.Mdata.Modify.F = 0;
        cmd_right.motor_send_data.Mdata.T = cmd_right.T * 256;
        cmd_right.motor_send_data.Mdata.W = cmd_right.W * 128;
        cmd_right.motor_send_data.Mdata.Pos = (int)((cmd_right.Pos / 6.2832f) * 16384.0f);
        cmd_right.motor_send_data.Mdata.K_P = cmd_right.K_P * 2048;
        cmd_right.motor_send_data.Mdata.K_W = cmd_right.K_W * 1024;
        cmd_right.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        cmd_right.motor_send_data.Mdata.LowHzMotorCmdByte = 0;
        cmd_right.motor_send_data.Mdata.Res[0] = cmd_right.Res;

        cmd_right.motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)(&cmd_right.motor_send_data), 7);

        memcpy(A1cmd_right, &cmd_right.motor_send_data, 34);

        HAL_UART_Transmit_DMA(&huart6, A1cmd_right, 34);
    }
}
