#include "niming.h"
#include "main.h"
#include "usart.h"
#include "encoder.h" 
uint8_t data_to_send[100];
uint8_t Data_Get[32];
/* ����Э�鶨�� */
#define ANO_HEADER1      0xAA
#define ANO_HEADER2      0xFF
#define ANO_ID_DATA_READ 0xE1    // ���ݶ�ȡID
#define ANO_ID_DATA_WRITE 0xE2   // ����д��ID
#define ANO_ID_CHECK     0x00    // У��֡ID
//??F1???4?uint16?????
void ANO_DT_Send_F1(uint16_t _a, uint16_t _b, uint16_t _c, uint16_t _d)
{
    uint8_t _cnt = 0;		//???
    uint8_t sumcheck = 0;  //???
    uint8_t addcheck = 0; //?????
    uint8_t i = 0;
	data_to_send[_cnt++] = 0xAA;//??
    data_to_send[_cnt++] = 0xFF;//????
    data_to_send[_cnt++] = 0xF1;//???
    data_to_send[_cnt++] = 8; //????
	//????????-?????????,?????????????,???????
	data_to_send[_cnt++] = BYTE0(_a);       
    data_to_send[_cnt++] = BYTE1(_a);
	
    data_to_send[_cnt++] = BYTE0(_b);
    data_to_send[_cnt++] = BYTE1(_b);
	
    data_to_send[_cnt++] = BYTE0(_c);
    data_to_send[_cnt++] = BYTE1(_c);
	
    data_to_send[_cnt++] = BYTE0(_d);
    data_to_send[_cnt++] = BYTE1(_d);
	 for ( i = 0; i < data_to_send[3]+4; i++)
    {
        sumcheck += data_to_send[i];//???
        addcheck += sumcheck;//????
    }
    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
	HAL_UART_Transmit(&huart1,data_to_send,_cnt,0xFFFF);//?????????
}
//,??F2???4?int16?????
void ANO_DT_Send_F2(int16_t _a, int16_t _b, int16_t _c, int16_t _d)   //F2?  4?  int16 ??
{
    uint8_t _cnt = 0;
    uint8_t sumcheck = 0; //???
    uint8_t addcheck = 0; //?????
    uint8_t i=0;
   data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0xF2;
    data_to_send[_cnt++] = 8; //????
	//????????-?????????,?????????????,???????
    data_to_send[_cnt++] = BYTE0(_a);
    data_to_send[_cnt++] = BYTE1(_a);
	
    data_to_send[_cnt++] = BYTE0(_b);
    data_to_send[_cnt++] = BYTE1(_b);
	
    data_to_send[_cnt++] = BYTE0(_c);
    data_to_send[_cnt++] = BYTE1(_c);
	
    data_to_send[_cnt++] = BYTE0(_d);
    data_to_send[_cnt++] = BYTE1(_d);
	
	  for ( i = 0; i < data_to_send[3]+4; i++)
    {
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }

    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
	
	HAL_UART_Transmit(&huart1,data_to_send,_cnt,0xFFFF);//?????????
}
//??F3???2?int16???1?int32?????
void ANO_DT_Send_F3(int16_t _a, int16_t _b, int32_t _c )   //F3?  2?  int16 ??   1?  int32  ??
{
    uint8_t _cnt = 0;
    uint8_t sumcheck = 0; //???
    uint8_t addcheck = 0; //?????
    uint8_t i=0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0xF3;
    data_to_send[_cnt++] = 8; //????
	//????????-?????????,?????????????,???????
    data_to_send[_cnt++] = BYTE0(_a);
    data_to_send[_cnt++] = BYTE1(_a);
	
    data_to_send[_cnt++] = BYTE0(_b);
    data_to_send[_cnt++] = BYTE1(_b);
	
    data_to_send[_cnt++] = BYTE0(_c);
    data_to_send[_cnt++] = BYTE1(_c);
    data_to_send[_cnt++] = BYTE2(_c);
    data_to_send[_cnt++] = BYTE3(_c);
	
	  for ( i = 0; i < data_to_send[3]+4; i++)
    {
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }

    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;

	HAL_UART_Transmit(&huart1,data_to_send,_cnt,0xFFFF);//?????????
}

// ���Ͳ���ֵ
void Ano_SentPar(uint16_t id, int32_t data)
{
    uint8_t cnt = 4, i = 0, sc = 0, ac = 0;
    
    data_to_send[0] = ANO_HEADER1;
    data_to_send[1] = ANO_HEADER2;
    data_to_send[2] = ANO_ID_DATA_READ;
    
    // �������
    data_to_send[cnt++] = BYTE0(id);
    data_to_send[cnt++] = BYTE1(id);
    
    data_to_send[cnt++] = BYTE0(data);
    data_to_send[cnt++] = BYTE1(data);
    data_to_send[cnt++] = BYTE2(data);
    data_to_send[cnt++] = BYTE3(data);
    
    data_to_send[3] = cnt - 4; // ���ݳ���
    
    // ����У��
    for(i = 0; i < cnt; i++)
    {
        sc += data_to_send[i];
        ac += sc;
    }
    
    data_to_send[cnt++] = sc;
    data_to_send[cnt++] = ac;
    
    HAL_UART_Transmit(&huart1, data_to_send, cnt, 0xFFFF);
}

// ����У��֡
void Ano_SentCheck(uint8_t id, uint8_t _sc, uint8_t _ac)
{
    uint8_t cnt = 4, i = 0, sc = 0, ac = 0;
    
    data_to_send[0] = ANO_HEADER1;
    data_to_send[1] = ANO_HEADER2;
    data_to_send[2] = ANO_ID_CHECK;
    
    // �������
    data_to_send[cnt++] = id;
    data_to_send[cnt++] = _sc;
    data_to_send[cnt++] = _ac;
    
    data_to_send[3] = cnt - 4; // ���ݳ���
    
    // ����У��
    for(i = 0; i < cnt; i++)
    {
        sc += data_to_send[i];
        ac += sc;
    }
    
    data_to_send[cnt++] = sc;
    data_to_send[cnt++] = ac;
    
    HAL_UART_Transmit(&huart1, data_to_send, cnt, 0xFFFF);
}

// ����Э�����ݽ�������
void Ano_Anl(uint8_t *Data_Pack)
{
    uint8_t sc = 0, ac = 0;
    
    for(uint8_t i = 0; i < Data_Pack[3] + 4; i++)
    {
        sc += Data_Pack[i];
        ac += sc;
    }
    
    // У����
    if(sc != Data_Pack[Data_Pack[3]+4] || ac != Data_Pack[Data_Pack[3]+5])
    {
        return;
    }
    
    if(Data_Pack[2] == ANO_ID_DATA_READ) // ������ȡID
    {
        uint16_t id = Data_Pack[4] + ((uint16_t)Data_Pack[5]<<8);
        
        switch(id)
        {
            case 1:
                Ano_SentPar(id,  Motor1Speed);        // �ٶ�
                break;
                
            case 2:
             //   Ano_SentPar(id, (int32_t)(num * 10)); // Ȧ���Ŵ�10������
                break;
                
            default:
                Ano_SentPar(id, 0);
                break;
        }
    }
    
    if(Data_Pack[2] == ANO_ID_DATA_WRITE) // ����д��ID
    {
        uint16_t id = Data_Pack[4] + ((uint16_t)Data_Pack[5]<<8);
        
        switch(id)
        {
            case 1:
                Motor1Speed = *(int32_t *)(&Data_Pack[6]);
               // rpm = Xianfu(rpm, Rpm_Max);
                break;
                
            case 2:
             //Ȧ��   num = (float)*(int32_t *)(&Data_Pack[6]) / 10;
                break;
        }
    }
    
    // ����У��
    Ano_SentCheck(Data_Pack[2], Data_Pack[Data_Pack[3]+4], Data_Pack[Data_Pack[3]+5]);
}

// �޷�����
int32_t Xianfu(int32_t value, int32_t max)
{
    if(value > max)
        return max;
    if(value < -max)
        return -max;
    return value;
}

// ����Э�����ݽ��մ���
void Ano_GetByte(uint8_t data)
{
    static uint8_t sta = 0, datalen = 0, datacnt = 0;
    
    if(sta == 0)
    {
        datalen = 0;
        datacnt = 0;
        
        Data_Get[0] = data;
        if(data == ANO_HEADER1)
            sta = 1;
    }
    else if(sta == 1) // ��ַ
    {
        Data_Get[1] = data;
        if(data == ANO_HEADER2)
            sta = 2;
        else
            sta = 0;
    }
    else if(sta == 2) // ID
    {
        Data_Get[2] = data;
        sta = 3;
    }
    else if(sta == 3) // ���ݳ���
    {
        sta = 4;
        Data_Get[3] = data;
        datalen = data;
    }
    else if(sta == 4) // ���ݻ���
    {
        Data_Get[4 + datacnt++] = data;
        if(datacnt >= datalen)
        {
            sta = 5;
        }
    }
    else if(sta == 5) // ��У��
    {
        Data_Get[4 + datacnt++] = data;
        sta = 6;
    }
    else if(sta == 6) // ����У��
    {
        sta = 0;
        Data_Get[4 + datacnt++] = data;
        
        Ano_Anl(Data_Get); // ���ݽ���
    }
}
