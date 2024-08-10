#include "vofa.h"
#include "single_sv.h"
#include "control.h"

void Float_to_Byte(float f, unsigned char byte[])
{
	FloatLongType fl;
	fl.fdata = f;
	byte[0] = (unsigned char)fl.ldata;
	byte[1] = (unsigned char)(fl.ldata >> 8);
	byte[2] = (unsigned char)(fl.ldata >> 16);
	byte[3] = (unsigned char)(fl.ldata >> 24);
}

void Send_Data(UART_HandleTypeDef *huart, float f)
{
	unsigned char byte[4] = {0};

	Float_to_Byte(f, byte);
	HAL_UART_Transmit(huart, (uint8_t *)&byte[0], 1, 0xffff);
	while (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX)
		;
	HAL_UART_Transmit(huart, (uint8_t *)&byte[1], 1, 0xffff);
	while (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX)
		;
	HAL_UART_Transmit(huart, (uint8_t *)&byte[2], 1, 0xffff);
	while (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX)
		;
	HAL_UART_Transmit(huart, (uint8_t *)&byte[3], 1, 0xffff);
	while (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX)
		;
}

void Send_Tail(UART_HandleTypeDef *huart)
{
	unsigned char byte[4] = {0x00, 0x00, 0x80, 0x7f};

	HAL_UART_Transmit(huart, (uint8_t *)&byte[0], 1, 0xffff);
	while (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX)
		;
	HAL_UART_Transmit(huart, (uint8_t *)&byte[1], 1, 0xffff);
	while (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX)
		;
	HAL_UART_Transmit(huart, (uint8_t *)&byte[2], 1, 0xffff);
	while (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX)
		;
	HAL_UART_Transmit(huart, (uint8_t *)&byte[3], 1, 0xffff);
	while (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX)
		;
}

// Add Send_Date
extern SV sv_2;
extern SOGI sg_U;
extern SOGI sg_I;
extern Transfer tran;
extern PID pi_D;
extern PID pi_Q;
extern PID pi_DC;
extern float Ui,U_alpha,U_beta,U_adc;
extern float Ii,I_alpha,I_beta;
extern float sin_sg,cos_sg,L,power_factor;
extern float Ud,Uq,Id,Iq,pi_d,pr_out,pi_out_D,pi_out_Q,pi_out_dc;
extern int oo;
void vodka_JustFloat_send(UART_HandleTypeDef *huart)
{
	
	/*待发送的数据*/
	Send_Data(huart,Ui);
	Send_Data(huart,Ii);
	Send_Data(huart,sg_U.vo);
	Send_Data(huart,sg_U.qvo);
	Send_Data(huart,Id);
	Send_Data(huart,Iq);
	Send_Data(huart,sin_sg);
	Send_Data(huart,cos_sg);
	Send_Data(huart,sg_I.vo);
	Send_Data(huart,sg_I.qvo);
  	Send_Data(huart,pi_out_D);
	Send_Data(huart,pi_out_Q);
	Send_Data(huart,pi_D.Integral);
	Send_Data(huart,pi_Q.Integral);
	Send_Data(huart,power_factor);
	Send_Data(huart,sv_2.CCR1);
	Send_Data(huart,sv_2.CCR2);
	
	/*尾帧*/
	Send_Tail(huart);
}

/**example**/
// while(1)
// {
// 	vodka_JustFloat_send(&huart1);
// }

#define RXSTRSIZE 256		  // 最大接收字节数
uint8_t rx_string[RXSTRSIZE]; // 接收字符串数组
uint8_t rx_cnt = 0;			  // 接收字符串计数
uint8_t rx_buff;			  // 接收缓存
uint8_t RxBuff;
unsigned char shujv[12];
int vofa_i = 0, vofa_I = 0;
float vofa_float;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t dat;
	if (huart->Instance == huart1.Instance)
	{
		
		// 以下是串口接收中断业务代码
		if (rx_cnt++ >= RXSTRSIZE - 1) // 溢出判断
		{
			rx_cnt = 0;
			memset(rx_string, 0x00, sizeof(rx_string));
		}
		else
		{
			dat = rx_buff; // 接收数据转存

			shujv[vofa_I++] = dat;

			if (dat == 0x0a)
			{

				for (vofa_i = 2, vofa_float = 0; vofa_i < vofa_I - 1; vofa_i++)
					vofa_float = 10 * vofa_float + (shujv[vofa_i] - '0');
				vofa_I = 0;

				if (shujv[1] == 0x3A)
				{
					switch (shujv[0])
					{
					// case 'S':my_dat.biaozhi =  shujv[2]  ; break;
					case 'p':
						// ZL_KP = vofa_float / 10.0;
						break;
//					case 'i':
//						spid.ki = vofa_float / 10.0;
//						break;
					case 'd':
						// a = vofa_float / 10.0;
						break;
//					case 'v':
//						spid.target = vofa_float;
//						break;
					}
				}
			}
			memset(rx_string, 0x00, sizeof(rx_string)); // 清空接收字符串
			rx_cnt = 0;									// 清空计数器
		}
	}

	HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_buff, 1);  // 再开启接收中断，若去掉只能接收一次
}
