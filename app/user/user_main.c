/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "user_config.h"
#include "driver/uart.h"

#include "c_types.h"
#include "eagle_soc.h"
#include "ip_addr.h"
#include "espconn.h"
#include "ets_sys.h"
#include "mem.h"

#include "osapi.h"

#include "espconn.h"
#include "os_type.h"
#include "ip_addr.h"

#include "user_interface.h"


#define USE_US_TIMER              //Ҫʹ�ú���os_timer_arm_us����Ҫ�ȶ���USE_US_TIMER




//    _______                            _________                           ____
//   |       |                          |         |                         |    |
//   | STM32 |���������������������������ڡ���������������������| ESP8266 |-----------UDP-----------| PC |
//   |_______|                          |_________|                         |____|
//

os_timer_t os_timer1;             //����һ�������ʱ������
os_timer_t os_timer2;             //����һ�������ʱ������
uint8_t led_state;                //����һ��LED��״̬�ı���

struct espconn user_udp_espconn;  //����һ��espconn�ṹ�����

uint8 uart_buf[1024]={0};         //����һ���������ݻ���ռ�

uint32 priv_param_start_sec;

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            priv_param_start_sec = 0x3C;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            priv_param_start_sec = 0x7C;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
            rf_cal_sec = 512 - 5;
            priv_param_start_sec = 0x7C;
            break;
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            priv_param_start_sec = 0xFC;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
            rf_cal_sec = 1024 - 5;
            priv_param_start_sec = 0x7C;
            break;
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            priv_param_start_sec = 0xFC;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            priv_param_start_sec = 0xFC;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            priv_param_start_sec = 0xFC;
            break;
        default:
            rf_cal_sec = 0;
            priv_param_start_sec = 0;
            break;
    }

    return rf_cal_sec;
}


void ICACHE_FLASH_ATTR
user_rf_pre_init(void)
{
}


/******************************************************************************
 * ������      : UDP���ͺ���
 * ����          : ͨ��UDPͨ��ת�����ڽ��յ�������
 * ����          : ��
 * ����ֵ      : ��
*******************************************************************************/
void ICACHE_FLASH_ATTR
uart_rx()
{

	//------------------------------------------------------------------------------------
	//����һ������������Ž��յ������ݳ���
    uint32 len = 0;


    //------------------------------------------------------------------------------------
    //������յ������ݳ���
    len = rx_buff_deq(uart_buf, 1024); //Ҫ���øú�������Ҫ�򿪴���buffʹ�ܺ꣨driver/uart.h�ļ���35�У�


    //------------------------------------------------------------------------------------
    //�жϽ��յ������ݳ��ȣ�������Ȳ�Ϊ�㣬˵�����յ������ݣ�Ȼ�����UDPת��
    if(len != 0)
    {

    	//------------------------------------------------------------------------------------
    	//UDPת��
    	espconn_sendto(&user_udp_espconn, uart_buf, len);


    	//------------------------------------------------------------------------------------
    	//UDPת��֮����ս�����
		os_memset(uart_buf,0,1024);


		//------------------------------------------------------------------------------------
		//ÿ�ζ�ʱ��ʱ�䵽��֮�󣬸��Ƶ�״̬ȡ��
		led_state = !led_state;


		//------------------------------------------------------------------------------------
		//�Ƶ�״̬ȡ���󣬸�ֵ���Ƶ�io��
		GPIO_OUTPUT_SET(GPIO_ID_PIN(2), led_state);

	}

}


/******************************************************************************
 * ������      : UDP���ջص�����
 * ����          : UDP���յ����ݺ�ص��˺���
 * ����          : ��
 * ����ֵ      : ��
*******************************************************************************/
void ICACHE_FLASH_ATTR
udp_recv_cb(void *arg,char *pdata,unsigned short len)
//���ջص��������壬ָ������ݳ���
{

	//------------------------------------------------------------------------------------
	//���ô��ڷ��ͺ������ͽ��յ�������
	uart0_tx_buffer((uint8 *)pdata,  len);

}


/******************************************************************************
 * ������      : UDPͨѶ��ʼ������
 * ����          : ��ʼ��UDPͨѶ
 * ����          : ��
 * ����ֵ      : ��
*******************************************************************************/
void ICACHE_FLASH_ATTR
udpserver_init(void)
{

	//------------------------------------------------------------------------------------
	//����һ����������ʾԶ��������IP��ַ
	//const char udp_remote_ip[4] = {192,168,4,2};
	const char udp_local_ip[4] = {192,168,4,1};


	//------------------------------------------------------------------------------------
	//���ṹ�����user_udp_espconn�ĳ�Ա��ֵ
	user_udp_espconn.type = ESPCONN_UDP;                                 //ͨ��Э������ΪUDP
	user_udp_espconn.proto.udp = (esp_udp *)os_zalloc(sizeof(esp_udp));  //����һ��esp_udp�ṹ�����
	//os_memcpy(user_udp_espconn.proto.udp->remote_ip, udp_remote_ip, 4);  //UDPͨѶԶ������IP
	os_memcpy(user_udp_espconn.proto.udp->local_ip, udp_local_ip, 4);  //UDPͨѶ����IP
	user_udp_espconn.proto.udp->local_port=9000;                         //UDPͨѶ���ض˿�
	user_udp_espconn.proto.udp->remote_port=9001;                        //UDPͨѶԶ�̶˿�

	//------------------------------------------------------------------------------------
	//ע��UDP���ջص�����
	espconn_regist_recvcb(&user_udp_espconn, udp_recv_cb);


	//------------------------------------------------------------------------------------
	//����UDPͨ�Žӿ�
	espconn_create(&user_udp_espconn);


	//------------------------------------------------------------------------------------
	//����UDPͨ�Žӿ���ɴ�ӡ��־
	os_printf("udp is ready!\r\n");

}


/******************************************************************************
 * ������      : AP�ȵ��ʼ��
 * ����          : ��ʼ��8266Ϊһ��AP�ȵ�
 * ����          : ��
 * ����ֵ      : ��
*******************************************************************************/
void ICACHE_FLASH_ATTR
softap_init(void)
{

	//------------------------------------------------------------------------------------
	//����һ��softap_config�ṹ�����
	struct softap_config config;


	//------------------------------------------------------------------------------------
	//���´����ʼ���ṹ�����config�ĳ�Ա
	os_strcpy(config.ssid, "MY_AP");           //ssid��AP�ȵ���
	os_strcpy(config.password, "1234567890");     //password��AP�ȵ�����

	config.ssid_len = os_strlen(config.ssid);     //ssid_len��AP�ȵ���ssid���ַ�������
	config.channel = 1;                           //channel���ŵ�ѡ��
	config.authmode = AUTH_WPA_PSK;               //authmode��AP�ȵ���ܷ�ʽ
	config.ssid_hidden = 0;                       //ssid_hidden���Ƿ�����AP�ȵ�����
	config.max_connection = 4;                    //max_connection��������Ӵ���
	config.beacon_interval = 100;                 //beacon_interval���ű���ʱ��


	//------------------------------------------------------------------------------------
	//����wifi_softap_set_config������Wi-Fi SoftAP�ӿڣ������ֵ�Flash
	wifi_softap_set_config(&config);

}


/******************************************************************************
 * ������      : �����ʱ����ʼ������
 * ����          : ��ʼ�������ʱ��
 * ����          : ��
 * ����ֵ      : ��
*******************************************************************************/
void ICACHE_FLASH_ATTR
os_timer1_init(void)
{

	//------------------------------------------------------------------------------------
	//����1���رն�ʱ��
	os_timer_disarm(&os_timer1);


	//------------------------------------------------------------------------------------
	//���ö�ʱ��,��ʱʱ�䵽�ᴥ���ص�����uart_rx����
	os_timer_setfn(&os_timer1, (os_timer_func_t	*)uart_rx, NULL);


	//------------------------------------------------------------------------------------
	//������ʱ�������ö�ʱ���Ϊ100us
	os_timer_arm_us(&os_timer1, 100,1);

}


/******************************************************************************
 * ������      : �û�ָʾ�Ƴ�ʼ������
 * ����          : ��ʼ���û�ָʾ��
 * ����          : ��
 * ����ֵ      : ��
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_led_init(void)
{

	//------------------------------------------------------------------------------------
	//�ܽŹ���ѡ����
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);       //ѡ��ܽ�2��ΪGPIO2�Ĺ��ܣ�led�ܽ�

	//------------------------------------------------------------------------------------
	//�ܽ�����ߵ͵�ƽ
	GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 1);                        //led�ܽ�Ĭ������ߣ��Ʋ���

}


/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_init(void)
{

	//------------------------------------------------------------------------------------
	//���³�ʼ����ʱ��
	system_timer_reinit();                                     //����Ҫʹ��΢�붨ʱ��ʱ����


	//------------------------------------------------------------------------------------
	//��ʼ������
	uart_init(BIT_RATE_921600, BIT_RATE_921600);


	//------------------------------------------------------------------------------------
	//��ʱ1ms
	os_delay_us(1000);


	//------------------------------------------------------------------------------------
	//��ӡSDK�汾��
	os_printf("\r\nSDK version: %s \n", system_get_sdk_version());


	//------------------------------------------------------------------------------------
	//�����û�ָʾ�Ƴ�ʼ����������ʼ���û�ָʾ��
	user_led_init();


	//------------------------------------------------------------------------------------
	//����AP�ȵ��ʼ����������ʼAP�ȵ�
	softap_init();


	//--------------------------------------------------------------------------------
	//��ʼ��UDPͨѶ
	udpserver_init();


	//------------------------------------------------------------------------------------
	//���������ʱ����ʼ������,��ʼ�������ʱ��
	os_timer1_init();

}

