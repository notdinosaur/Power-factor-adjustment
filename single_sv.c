#include "single_sv.h"
#include "math.h"
#include "main.h"
#include "tim.h"

void single_sv(SV *sv,float U��,float Udc)
{
	//float L,cos;
	//��������
	sv->U�� = U��;
	//sv->U�� = U��;
	sv->Udc = Udc;
	sv->Ts = 1;
	
	//L = sqrt(sv->U��*sv->U��+sv->U��*sv->U��);
	
	//�����ж�
	if(U��>0)
	{
		sv->sector = 1;
	}
	else
	{
		sv->sector = 2;
	}
	
	//�޷�
	if(sv->U��>sv->Udc)
	{
		sv->U�� = sv->Udc;
	}

	if(sv->U��<-1*sv->Udc)
	{
		sv->U�� = -1*sv->Udc;
	}
	
	//ʱ�����
	switch(sv->sector)
	{
		case 1:
			sv->T1 = (sv->U��/sv->Udc)/sv->Ts;
			sv->T0 = sv->Ts - sv->T1;
			break;
		case 2:
			sv->T2 = -(sv->U��/sv->Udc)/sv->Ts;
			sv->T0 = sv->Ts - sv->T2;
			break;	
	}
	
	// //���ʽ����
	switch(sv->sector)
	{
		case 1:
			sv->CCR1 = ((sv->T1 + sv->T0/2.0f)/sv->Ts) * 4666;
			sv->CCR2 = (sv->T0/2.0f)/sv->Ts * 4666;
			break;
		case 2:
			sv->CCR1 = (sv->T0/2.0f)/sv->Ts * 4666;
			sv->CCR2 = ((sv->T2 + sv->T0/2.0f)/sv->Ts) * 4666;
			break;			
	}


	// if(sv->CCR1 > 4666) sv->CCR1 = 4666;
	// if(sv->CCR1 < 0) sv->CCR1 = 0;
	
	// if(sv->CCR2 > 4666) sv->CCR2 = 4666;
	// if(sv->CCR2 < 0) sv->CCR2 = 0;


}




void SPWM(float Uref,float Umax,float Uac)
{
	float sector;
	int CCR1,CCR2;

	if(Uac>0.0f)	sector = 1;
	if(Uac<0.0f)	sector = 2;

	if(sector == 1)
	{
		CCR1 = 0.0f; 
		CCR2 = (Uref/Umax) * 4666;
	}

	if(sector == 2)
	{
		CCR1 = (Uref/Umax) * 4666; 
		CCR2 = 0.0f;
	}

	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,CCR1);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,CCR2);

}


