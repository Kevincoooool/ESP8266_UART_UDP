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


#define USE_US_TIMER              //要使用函数os_timer_arm_us，需要先定义USE_US_TIMER




//    _______                            _________                           ____
//   |       |                          |         |                         |    |
//   | STM32 |————————————串口———————————| ESP8266 |-----------UDP-----------| PC |
//   |_______|                          |_________|                         |____|
//

os_timer_t os_timer1;             //声明一个软件定时器变量
os_timer_t os_timer2;             //声明一个软件定时器变量
uint8_t led_state;                //声明一个LED灯状态的变量

struct espconn user_udp_espconn;  //声明一个espconn结构体变量

uint8 uart_buf[1024]={0};         //开辟一个串口数据缓存空间

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
 * 函数名      : UDP发送函数
 * 描述          : 通过UDP通信转发串口接收到的数据
 * 参数          : 无
 * 返回值      : 无
*******************************************************************************/
void ICACHE_FLASH_ATTR
uart_rx()
{

	//------------------------------------------------------------------------------------
	//声明一个变量用来存放接收到的数据长度
    uint32 len = 0;


    //------------------------------------------------------------------------------------
    //计算接收到的数据长度
    len = rx_buff_deq(uart_buf, 1024); //要调用该函数，需要打开串口buff使能宏（driver/uart.h文件第35行）


    //------------------------------------------------------------------------------------
    //判断接收到的数据长度，如果长度不为零，说明接收到了数据，然后进行UDP转发
    if(len != 0)
    {

    	//------------------------------------------------------------------------------------
    	//UDP转发
    	espconn_sendto(&user_udp_espconn, uart_buf, len);


    	//------------------------------------------------------------------------------------
    	//UDP转发之后清空接收区
		os_memset(uart_buf,0,1024);


		//------------------------------------------------------------------------------------
		//每次定时器时间到了之后，给灯的状态取反
		led_state = !led_state;


		//------------------------------------------------------------------------------------
		//灯的状态取反后，赋值给灯的io口
		GPIO_OUTPUT_SET(GPIO_ID_PIN(2), led_state);

	}

}


/******************************************************************************
 * 函数名      : UDP接收回调函数
 * 描述          : UDP接收到数据后回调此函数
 * 参数          : 无
 * 返回值      : 无
*******************************************************************************/
void ICACHE_FLASH_ATTR
udp_recv_cb(void *arg,char *pdata,unsigned short len)
//接收回调函数定义，指针和数据长度
{

	//------------------------------------------------------------------------------------
	//调用串口发送函数发送接收到的数据
	uart0_tx_buffer((uint8 *)pdata,  len);

}


/******************************************************************************
 * 函数名      : UDP通讯初始化函数
 * 描述          : 初始化UDP通讯
 * 参数          : 无
 * 返回值      : 无
*******************************************************************************/
void ICACHE_FLASH_ATTR
udpserver_init(void)
{

	//------------------------------------------------------------------------------------
	//声明一个常量，表示远程主机的IP地址
	//const char udp_remote_ip[4] = {192,168,4,2};
	const char udp_local_ip[4] = {192,168,4,1};


	//------------------------------------------------------------------------------------
	//给结构体变量user_udp_espconn的成员赋值
	user_udp_espconn.type = ESPCONN_UDP;                                 //通信协议类型为UDP
	user_udp_espconn.proto.udp = (esp_udp *)os_zalloc(sizeof(esp_udp));  //声明一个esp_udp结构体变量
	//os_memcpy(user_udp_espconn.proto.udp->remote_ip, udp_remote_ip, 4);  //UDP通讯远程主机IP
	os_memcpy(user_udp_espconn.proto.udp->local_ip, udp_local_ip, 4);  //UDP通讯本地IP
	user_udp_espconn.proto.udp->local_port=9000;                         //UDP通讯本地端口
	user_udp_espconn.proto.udp->remote_port=9001;                        //UDP通讯远程端口

	//------------------------------------------------------------------------------------
	//注册UDP接收回调函数
	espconn_regist_recvcb(&user_udp_espconn, udp_recv_cb);


	//------------------------------------------------------------------------------------
	//创建UDP通信接口
	espconn_create(&user_udp_espconn);


	//------------------------------------------------------------------------------------
	//创建UDP通信接口完成打印日志
	os_printf("udp is ready!\r\n");

}


/******************************************************************************
 * 函数名      : AP热点初始化
 * 描述          : 初始化8266为一个AP热点
 * 参数          : 无
 * 返回值      : 无
*******************************************************************************/
void ICACHE_FLASH_ATTR
softap_init(void)
{

	//------------------------------------------------------------------------------------
	//声明一个softap_config结构体变量
	struct softap_config config;


	//------------------------------------------------------------------------------------
	//以下代码初始化结构体变量config的成员
	os_strcpy(config.ssid, "MY_AP");           //ssid：AP热点名
	os_strcpy(config.password, "1234567890");     //password：AP热点密码

	config.ssid_len = os_strlen(config.ssid);     //ssid_len：AP热点名ssid的字符串长度
	config.channel = 1;                           //channel：信道选择
	config.authmode = AUTH_WPA_PSK;               //authmode：AP热点加密方式
	config.ssid_hidden = 0;                       //ssid_hidden：是否隐藏AP热点密码
	config.max_connection = 4;                    //max_connection：最大连接次数
	config.beacon_interval = 100;                 //beacon_interval：信标间隔时间


	//------------------------------------------------------------------------------------
	//调用wifi_softap_set_config来设置Wi-Fi SoftAP接口，并保持到Flash
	wifi_softap_set_config(&config);

}


/******************************************************************************
 * 函数名      : 软件定时器初始化函数
 * 描述          : 初始化软件定时器
 * 参数          : 无
 * 返回值      : 无
*******************************************************************************/
void ICACHE_FLASH_ATTR
os_timer1_init(void)
{

	//------------------------------------------------------------------------------------
	//步骤1：关闭定时器
	os_timer_disarm(&os_timer1);


	//------------------------------------------------------------------------------------
	//设置定时器,定时时间到会触发回调函数uart_rx工作
	os_timer_setfn(&os_timer1, (os_timer_func_t	*)uart_rx, NULL);


	//------------------------------------------------------------------------------------
	//启动定时器，设置定时间隔为100us
	os_timer_arm_us(&os_timer1, 100,1);

}


/******************************************************************************
 * 函数名      : 用户指示灯初始化函数
 * 描述          : 初始化用户指示灯
 * 参数          : 无
 * 返回值      : 无
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_led_init(void)
{

	//------------------------------------------------------------------------------------
	//管脚功能选择函数
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);       //选择管脚2作为GPIO2的功能，led管脚

	//------------------------------------------------------------------------------------
	//管脚输出高低电平
	GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 1);                        //led管脚默认输出高，灯不亮

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
	//重新初始化定时器
	system_timer_reinit();                                     //当需要使用微秒定时器时调用


	//------------------------------------------------------------------------------------
	//初始化串口
	uart_init(BIT_RATE_921600, BIT_RATE_921600);


	//------------------------------------------------------------------------------------
	//延时1ms
	os_delay_us(1000);


	//------------------------------------------------------------------------------------
	//打印SDK版本号
	os_printf("\r\nSDK version: %s \n", system_get_sdk_version());


	//------------------------------------------------------------------------------------
	//调用用户指示灯初始化函数，初始化用户指示灯
	user_led_init();


	//------------------------------------------------------------------------------------
	//调用AP热点初始化函数，初始AP热点
	softap_init();


	//--------------------------------------------------------------------------------
	//初始化UDP通讯
	udpserver_init();


	//------------------------------------------------------------------------------------
	//调用软件定时器初始化函数,初始化软件定时器
	os_timer1_init();

}

