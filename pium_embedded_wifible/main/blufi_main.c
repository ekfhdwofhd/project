// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "bt.h"

#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "blufi_demo.h"



#define ONE_MINUTE 60


//////led////////////



#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define LEDC_IO_0   17
#define LEDC_IO_1   18
#define LEDC_IO_2   19


#define MINIMUM_DIFFUSING_INTERVAL_MS 500 //0.5second
//////led end////////

////////////////////aws////////////////////




#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_event.h"
#include "nvs.h"

#include "certs.h"
#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

Timer wifi_disconnect_timeout;

#define TOPIC_PUBLISH "toServer"

#define PUBLISH_INTERVAL_SEC 150
#define YIELD_INTERVAL_MS 4000

#define STORAGE_NAMESPACE "storage"

static EventGroupHandle_t wifi_event_group;

  size_t required_size_ssid = 0;
  size_t required_size_pass = 0;

    nvs_handle my_handle;
    esp_err_t err;
    
static int wifi_disconnect_count = 0;

/////////////////////////////////////////////////////aws end

//static bool isDiffusing = false;

static bool newSchedule = false;
static bool cancelSchedule = false;
static bool requestSchedule = false;
uint32_t schedule_bitmap;
static char pattern1; 
static char pattern2; 
static char pattern3;
static char scheduleDuration;



static void blufi_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param);

#define BLUFI_DEVICE_NAME            "BLUFI_DEVICE"
static uint8_t blufi_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
};

//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
static esp_ble_adv_data_t blufi_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x100,
    .max_interval = 0x100,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = blufi_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t blufi_adv_params = {
    .adv_int_min        = 0x100,
    .adv_int_max        = 0x100,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define WIFI_LIST_NUM   10

static wifi_config_t sta_config;
static wifi_config_t ap_config;

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

/* store the station info for send back to phone */
static bool gl_sta_connected = false;
static uint8_t gl_sta_bssid[6];
static uint8_t gl_sta_ssid[32];
static int gl_sta_ssid_len;


void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) {
    IOT_WARN("MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if (NULL == pClient) {
        return;
    }

    IOT_UNUSED(data);

    if (aws_iot_is_autoreconnect_enabled(pClient)) {
        IOT_INFO("Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        IOT_WARN("Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if (NETWORK_RECONNECTED == rc) {
        IOT_WARN("Manual Reconnect Successful");
        } else {
        IOT_WARN("Manual Reconnect Failed - %d", rc);
        }
    }
}


void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                  IoT_Publish_Message_Params *params, void *pData) {
    IOT_UNUSED(pData);
    IOT_UNUSED(pClient);
    IOT_INFO("Subscribe callback\n");
    const char *in_payload = (char*) params->payload;

    char command[2]={0,};
    char repeat[3]={0,};
    char schedule[9]={0,};
    char params1[3]={0,};
    char params2[3]={0,};
    char params3[3]={0,};
    char com;

    strncpy(command, &in_payload[0], 1);
    com = (char)strtol(command, NULL, 16);
    
    switch (com) {
        case 0xA : 
            strncpy(params1, &in_payload[1], 2);
            strncpy(params2, &in_payload[3], 2);
            strncpy(params3, &in_payload[5], 2);
            
            pattern1 = (char)strtol(params1, NULL, 16);
            pattern2 = (char)strtol(params2, NULL, 16);
            pattern3 = (char)strtol(params3, NULL, 16);

            newSchedule=true;
            scheduleDuration=0;
            IOT_INFO("Command A:\nFlaskA: %s\nFlaskB: %s\nFlaskC: %s\n", params1, params2, params3);
            break;
        
        case 0xB :       
            strncpy(repeat, &in_payload[1], 2);
            strncpy(schedule, &in_payload[3], 8);
            strncpy(params1, &in_payload[11], 2);
            strncpy(params2, &in_payload[13], 2);
            strncpy(params3, &in_payload[15], 2);

            pattern1 = (char)strtol(params1, NULL, 16);
            pattern2 = (char)strtol(params2, NULL, 16);
            pattern3 = (char)strtol(params3, NULL, 16);	
            
            scheduleDuration = (char)strtol(repeat, NULL, 10);
            schedule_bitmap = (uint32_t)strtoul(schedule, NULL, 16);

            newSchedule = true;	
            IOT_INFO("Command B:\nSchedule Duration: %d\nSchedule Pattern:0x%X\nFlaskA: %s\nFlaskB: %s\nFlaskC: %s\n", scheduleDuration, schedule_bitmap, params1, params2, params3);
            break;

        case 0xC :
            cancelSchedule = true;
            IOT_INFO("Command C:\nCanel Current Schedule");
            break;
        
		case 0xD ://테스트용 , nvs reset and reboot
			IOT_INFO("Command D:\n nvs_erase_all");
			nvs_erase_all(my_handle);
			esp_restart();
			break;    
			       
        default: IOT_INFO("Wrong command!!");
            break;
            
    }
}

/**
 *  Main task:
 *    - Started when wifi gets IP address.
 *    - Initialises, connects to AWS IoT Client. 
 *    - Subscribes to topic specified by TOPIC_SUBSCRIBE. 
 *    - Publishes to topic TOPIC_PUBLISH every PUBLISH_INTERVAL_SEC. (Currently publishes meaningless information). 
 */
 
 
 
 static void aws_iot_mqtt_task(void *pvParameters) 
{
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    fprintf(stderr, "Connected to AP\n");

    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;

    IOT_INFO("AWS IoT SDK Version %d.%d.%d-%s\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = AWS_IOT_MQTT_HOST;
    mqttInitParams.port = AWS_IOT_MQTT_PORT;
    mqttInitParams.pRootCALocation = "";
    mqttInitParams.pDeviceCertLocation = "";
    mqttInitParams.pDevicePrivateKeyLocation = "";
    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 20000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;

    IoT_Error_t rc = FAILURE;
    AWS_IoT_Client client;
    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if (SUCCESS != rc) {
        IOT_ERROR("aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    uint8_t my_mac[6];
    ESP_ERROR_CHECK( esp_wifi_get_mac(ESP_IF_WIFI_STA, my_mac) );

    char deviceId[13];
    sprintf(deviceId, "%02x%02x%02x%02x%02x%02x", my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);

    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;
    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    connectParams.pClientID = deviceId;
    connectParams.clientIDLen = (uint16_t) strlen(deviceId);
    connectParams.isWillMsgPresent = false;

    IOT_INFO("Connecting...");
    rc = aws_iot_mqtt_connect(&client, &connectParams);
    if (SUCCESS != rc) {
        IOT_ERROR("Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
        abort();
    }

    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if (SUCCESS != rc) {
        IOT_ERROR("Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    const char *topicSubscribe = deviceId;
    const char *topicPublish = TOPIC_PUBLISH;

    printf("My Device ID is: %s\n", deviceId);
    IOT_INFO("Subscribing...");
    
    rc = aws_iot_mqtt_subscribe(&client, topicSubscribe, strlen(topicSubscribe), QOS0, iot_subscribe_callback_handler, NULL);
    if (SUCCESS != rc) {
        IOT_ERROR("Error subscribing : %d", rc)
    }

    IOT_DEBUG("Creating cPayload");
    char cPayload[100];
    uint32_t payloadCount = 0;

    IoT_Publish_Message_Params paramsQOS0;
    paramsQOS0.qos = QOS0;
    paramsQOS0.payload = (void *) cPayload;
    paramsQOS0.isRetained = 0;

    uint32_t reconnectAttempts = 0;
    uint32_t reconnectedCount = 0;

    while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc)) {
        // IOT_DEBUG("Top of loop: payloadCount=%d, reconnectAttempts=%d, reconnectedCount=%d\n", payloadCount, reconnectAttempts, reconnectedCount);

        // Max time the yield function will wait for read messages
        IOT_INFO("Yielding...\n");
        rc = aws_iot_mqtt_yield(&client, YIELD_INTERVAL_MS);

        if (NETWORK_ATTEMPTING_RECONNECT == rc) {
        reconnectAttempts++;
        IOT_DEBUG("NETWORK_ATTEMPTING_RECONNECT... #%d\n", reconnectAttempts);
        // If the client is attempting to reconnect we will skip the rest of the loop.
        continue;
        }

        if (NETWORK_RECONNECTED == rc) {
        reconnectedCount++;
        IOT_DEBUG(stderr, "NETWORK_RECONNECTED Reconnected...#%d\n", reconnectAttempts);
        }

        if (requestSchedule) {
            sprintf(cPayload, "{\"deviceId\":\"%s\"}", deviceId);
            paramsQOS0.payloadLen = strlen(cPayload);
            rc = aws_iot_mqtt_publish(&client, topicPublish, strlen(topicPublish), &paramsQOS0);
            
            if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
                IOT_DEBUG("QOS0 publish ack not received.\n");
                rc = SUCCESS;
            }

            if (SUCCESS != rc) {
                IOT_ERROR("An error occurred in the loop.\n");
            } else {
                IOT_INFO("Publish done\n");
                requestSchedule = false;
            }
        }
    }

    IOT_ERROR("Escaped loop...\n");
    abort();
}
 


static void initialise_gpio(void) 
{
    gpio_pad_select_gpio(5);
    gpio_set_direction(5, GPIO_MODE_OUTPUT);
    // gpio_pad_select_gpio(17);
    // gpio_set_direction(17, GPIO_MODE_OUTPUT);
    ledc_timer_config_t ledc_timer = {
        //set timer counter bit number
        .bit_num = 8,
        //set frequency of pwm
        .freq_hz = 113000, //was 5000
        //timer mode,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        //timer index
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        //set LEDC channel 0
        .channel = LEDC_CHANNEL_0,
        //set the duty for initialization.(duty range is 0 ~ ((2**bit_num)-1)
        .duty = 128,
        //GPIO number
        .gpio_num = LEDC_IO_0,
        //GPIO INTR TYPE, as an example, we enable fade_end interrupt here.
        .intr_type = LEDC_INTR_DISABLE,
        //set LEDC mode, from ledc_mode_t
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        //set LEDC timer source, if different channel use one timer,
        //the frequency and bit_num of these channels should be the same
        .timer_sel = LEDC_TIMER_0
    };
    //set the configuration
    ledc_channel_config(&ledc_channel);

    // //config ledc channel1
    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = LEDC_IO_1;
    ledc_channel_config(&ledc_channel);
    // //config ledc channel2
    ledc_channel.channel = LEDC_CHANNEL_2;
    ledc_channel.gpio_num = LEDC_IO_2;
    ledc_channel_config(&ledc_channel);

    //initialize fade service.
    ledc_fade_func_install(0);  
   
    
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);//128로 초기화 되어 있기 때문에 0으로 꺼놔야한다.
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 0);
	
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);

	gpio_set_level(5, 0);

}

static void control_led(){
  for(int i = 0; i < 8; ++i) {
     //   printf("LEDC set duty without fade: ON\n");
        // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 128);
        // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        // while(1) {
        //     vTaskDelay(500 / portTICK_PERIOD_MS);
        //     printf("Hello\n");
        // }
        if((pattern1>>(7-i)) & 1){
			ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 128);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
		}else{
			ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
		}
		 if((pattern2>>(7-i)) & 1){
			 ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 128);
			 ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
		 }else{
			 ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);	
			 ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);		
		 }
		 if((pattern3>>(7-i)) & 1){
			 ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 128);
			 ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
		 }else{
			 ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 0);	
			 ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);	 
		 }
        gpio_set_level(5, 1);
        vTaskDelay(MINIMUM_DIFFUSING_INTERVAL_MS / portTICK_PERIOD_MS);
  }
       // printf("LEDC set duty without fade: OFF\n");
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 0);
        
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);

        gpio_set_level(5, 0);
}

static esp_err_t event_handler_blemode(void *ctx, system_event_t *event)//wifi 연결될때까지 무한으로 돈다.
{
    wifi_mode_t mode;

    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP: {//wifi connect. 
        esp_blufi_extra_info_t info;

        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        esp_wifi_get_mode(&mode);

        memset(&info, 0, sizeof(esp_blufi_extra_info_t));
        memcpy(info.sta_bssid, gl_sta_bssid, 6);
        info.sta_bssid_set = true;
        info.sta_ssid = gl_sta_ssid;
        info.sta_ssid_len = gl_sta_ssid_len;
        esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
       
         err = nvs_set_str(my_handle, "ssid", (char *)sta_config.sta.ssid); //nvs setting.
		 err = nvs_set_str(my_handle, "pass", (char *)sta_config.sta.password);
		
          nvs_commit(my_handle);//Write any pending changes to non-volatile storage
          nvs_close(my_handle);
          fflush(stdout);
			if(required_size_ssid==0){
			 for (int i = 5; i >= 0; i--) {
				printf("Restarting in %d seconds...\n", i);
				vTaskDelay(1000 / portTICK_RATE_MS);
			}
				esp_restart();//reboot.
			}    
        break;
    }
    case SYSTEM_EVENT_STA_CONNECTED:
        gl_sta_connected = true;// gl_sta_connected 이게 wifi연결 유무파악!!!
        memcpy(gl_sta_bssid, event->event_info.connected.bssid, 6);
        memcpy(gl_sta_ssid, event->event_info.connected.ssid, event->event_info.connected.ssid_len);
        gl_sta_ssid_len = event->event_info.connected.ssid_len;
        
        break; 
    case SYSTEM_EVENT_STA_DISCONNECTED://wifi disconnect.
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        printf("disconnect!!!!!!!!!!!!!!!!!\n");
        gl_sta_connected = false;
        memset(gl_sta_ssid, 0, 32);
        memset(gl_sta_bssid, 0, 6);
        gl_sta_ssid_len = 0;
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_START:
        esp_wifi_get_mode(&mode);

        /* TODO: get config or information of softap, then set to report extra_info */
        if (gl_sta_connected) {  
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, NULL);
        } else {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);
        }
        break;
    default:
        break;
    }
    return ESP_OK;
}

static esp_err_t event_handler_wifimode(void *ctx, system_event_t *event)
{	
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP://wifi connent
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            wifi_disconnect_count=0;
            printf("connect!!!!!wifi_disconnect_count:%d\n",wifi_disconnect_count);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED://wifi disconnect			
			wifi_disconnect_count++;
			printf("disconnect!!!!wifi_disconnect_count:%d\n",wifi_disconnect_count);
			if(wifi_disconnect_count==1)
				countdown_sec(&wifi_disconnect_timeout,30);
			
			if(has_timer_expired(&wifi_disconnect_timeout))//이걸 그냥 wifi_disconnect_count 값으로 해도 될듯.
			{
				for (int i = 5; i >= 0; i--) {
					printf("wifi disconnected. Restarting in %d seconds...\n", i);
					vTaskDelay(1000 / portTICK_RATE_MS);
				}
				nvs_erase_all(my_handle);
				esp_restart();
			}
								
            /* This is a workaround as ESP32 WiFi libs don't currently
                auto-reassociate. */
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void initialise_wifi_blemode(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler_blemode, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}
static void initialise_wifi_wifimode(char * ssid, char * password)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler_wifimode, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
 
    wifi_config_t wifi_config;
     memset(&wifi_config,0,sizeof(wifi_config));//initialize
	     strcpy((char *)wifi_config.sta.ssid, (char *)ssid);    // 그냥 배열값 대입하면 안된다!! 
	    // wifi_config.sta.ssid[required_size_ssid] = '\0';     
	     strcpy((char *)wifi_config.sta.password, (char *)password);
	    // wifi_config.sta.password[required_size_pass] = '\0'; 
    //~ wifi_config_t wifi_config = {
        //~ .sta = {
        //~ .ssid =, 
        //~ .password = ,
        //~ },
    //~ };
    fprintf(stderr, "Setting WiFi configuration SSID %s...\n", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() ); 
   
}

static esp_blufi_callbacks_t blufi_callbacks = {
    .event_cb = blufi_event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func = blufi_aes_encrypt,
    .decrypt_func = blufi_aes_decrypt,
    .checksum_func = blufi_crc_checksum,
};

static void blufi_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param)
{
    /* actually, should post to blufi_task handle the procedure,
     * now, as a demo, we do simplely */
    switch (event) {
    case ESP_BLUFI_EVENT_INIT_FINISH:
        BLUFI_INFO("BLUFI init finish\n");
        esp_ble_gap_set_device_name(BLUFI_DEVICE_NAME);
        esp_ble_gap_config_adv_data(&blufi_adv_data);
        break;
    case ESP_BLUFI_EVENT_DEINIT_FINISH:
        BLUFI_INFO("BLUFI init finish\n");
        break;
    case ESP_BLUFI_EVENT_BLE_CONNECT:
        BLUFI_INFO("BLUFI ble connect\n");
        esp_ble_gap_stop_advertising();
        blufi_security_deinit();
        blufi_security_init();
        break;
    case ESP_BLUFI_EVENT_BLE_DISCONNECT:
        BLUFI_INFO("BLUFI ble disconnect\n");
        esp_ble_gap_start_advertising(&blufi_adv_params);
        break;
    case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
        BLUFI_INFO("BLUFI Set WIFI opmode %d\n", param->wifi_mode.op_mode);
        ESP_ERROR_CHECK( esp_wifi_set_mode(param->wifi_mode.op_mode) );
        break;
    case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
        BLUFI_INFO("BLUFI requset wifi connect to AP\n");
        esp_wifi_connect();
        break;
    case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
        BLUFI_INFO("BLUFI requset wifi disconnect from AP\n");
        esp_wifi_disconnect();
        break;
    case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {
        wifi_mode_t mode;
        esp_blufi_extra_info_t info;

        esp_wifi_get_mode(&mode);

        if (gl_sta_connected ) {  
            memset(&info, 0, sizeof(esp_blufi_extra_info_t));
            memcpy(info.sta_bssid, gl_sta_bssid, 6);
            info.sta_bssid_set = true;
            info.sta_ssid = gl_sta_ssid;
            info.sta_ssid_len = gl_sta_ssid_len;
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
        } else {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);
        }
        BLUFI_INFO("BLUFI get wifi status from AP\n");

        break;
    }
    case ESP_BLUFI_EVENT_DEAUTHENTICATE_STA:
        /* TODO */
        break;
	case ESP_BLUFI_EVENT_RECV_STA_BSSID:
        memcpy(sta_config.sta.bssid, param->sta_bssid.bssid, 6);
        sta_config.sta.bssid_set = 1;
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("Recv STA BSSID %s\n", sta_config.sta.ssid);
        
        break;
	case ESP_BLUFI_EVENT_RECV_STA_SSID:
        strncpy((char *)sta_config.sta.ssid, (char *)param->sta_ssid.ssid, param->sta_ssid.ssid_len);
        sta_config.sta.ssid[param->sta_ssid.ssid_len] = '\0';
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        BLUFI_INFO("Recv STA SSID %s\n", sta_config.sta.ssid);
        
        break;
	case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
        strncpy((char *)sta_config.sta.password, (char *)param->sta_passwd.passwd, param->sta_passwd.passwd_len);
        sta_config.sta.password[param->sta_passwd.passwd_len] = '\0';    
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);      
        BLUFI_INFO("Recv STA PASSWORD %s\n", sta_config.sta.password);
       
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_SSID:
        strncpy((char *)ap_config.ap.ssid, (char *)param->softap_ssid.ssid, param->softap_ssid.ssid_len);
        ap_config.ap.ssid_len = param->softap_ssid.ssid_len;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP SSID %s, ssid len %d\n", ap_config.ap.ssid, ap_config.ap.ssid_len);
        
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD:
        strncpy((char *)ap_config.ap.password, (char *)param->softap_passwd.passwd, param->softap_passwd.passwd_len);
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP PASSWORD %s\n", ap_config.ap.password);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_MAX_CONN_NUM:
        if (param->softap_max_conn_num.max_conn_num > 4) {
            return;
        }
        ap_config.ap.max_connection = param->softap_max_conn_num.max_conn_num;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP MAX CONN NUM %d\n", ap_config.ap.max_connection);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_AUTH_MODE:
        if (param->softap_auth_mode.auth_mode >= WIFI_AUTH_MAX) {
            return;
        }
        ap_config.ap.authmode = param->softap_auth_mode.auth_mode;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP AUTH MODE %d\n", ap_config.ap.authmode);
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_CHANNEL:
        if (param->softap_channel.channel > 13) {
            return;
        }
        ap_config.ap.channel = param->softap_channel.channel;
        esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        BLUFI_INFO("Recv SOFTAP CHANNEL %d\n", ap_config.ap.channel);
        break;
	case ESP_BLUFI_EVENT_RECV_USERNAME:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CA_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
        /* Not handle currently */
        break;;
	case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
        /* Not handle currently */
        break;
    default:
        break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&blufi_adv_params);
        break;
    default:
        break;
    }
}

static void schedule_control(void *pvParameters) 
{
    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);   //Delay is necessary for WatchDog Timer error.
        if (newSchedule) {
            if (scheduleDuration == 0) {        // type A defusing 
                control_led();
                newSchedule = false;
            } else {
				Timer sendSomething;
                for (int i = 0; i < scheduleDuration && i < 30; ++i) {
                    if (cancelSchedule) {
                        cancelSchedule = false;
                        break;
                    }
                 	countdown_sec(&sendSomething,ONE_MINUTE);   // countdown 60 second
                    if( (schedule_bitmap >> (29-i) & 1) == 1 ) {			
                        control_led();
                    }                    
                    while(!(cancelSchedule || has_timer_expired(&sendSomething)))   // wait until the countdown is over. If the cancel command comes in during waiting, it exits.
						vTaskDelay(100 / portTICK_PERIOD_MS); 	 //Delay is necessary for WatchDog Timer error																						
                }
                newSchedule = false;
                if (scheduleDuration > 30) {
                    requestSchedule = true;
                }
            }
        }
    }
}
static void bluetooth_mode()//ble mode
{
	esp_err_t ret;
	nvs_flash_init();
	initialise_wifi_blemode();
	esp_bt_controller_init();
	ret = esp_bluedroid_init();
	
	if (ret) {
		BLUFI_ERROR("%s init bluedroid failed\n", __func__);
		return;
	}		
	ret = esp_bluedroid_enable();
	if (ret) {
		BLUFI_ERROR("%s init bluedroid failed\n", __func__);
		return;
	}
	
	BLUFI_INFO("BD ADDR: "ESP_BD_ADDR_STR"\n", ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));
	BLUFI_INFO("BLUFI VERSION %04x\n", esp_blufi_get_version());
	blufi_security_init();
	esp_ble_gap_register_callback(gap_event_handler);
	esp_blufi_register_callbacks(&blufi_callbacks);
	esp_blufi_profile_init();
}

static void wifi_mode()//wifi mode
{
	char* my_ssid = malloc(required_size_ssid);
	char* my_pass = malloc(required_size_pass);
	err = nvs_get_str(my_handle, "ssid", my_ssid, &required_size_ssid);//get ssid
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
	err = nvs_get_str(my_handle, "pass", my_pass, &required_size_pass); //get pass
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
	//~ if (err == ESP_OK) {
		//~ printf("after restarting!!!! my_ssid: %s\n", my_ssid);
		//~ printf("after restarting!!!! required_size_ssid: %d\n", required_size_ssid);
		//~ printf("after restarting!!!! my_pass: %s\n", my_pass);
		//~ printf("after restarting!!!! required_size_pass: %d\n", required_size_pass);
	//~ }
	initialise_wifi_wifimode(my_ssid,my_pass);
	initialise_gpio();
	xTaskCreate(&aws_iot_mqtt_task, "AWS IoT", 8192*2, NULL, 5, NULL);
	xTaskCreate(&schedule_control, "Scheduler",512, NULL, 5, NULL);   		
}

void app_main()
{
    nvs_flash_init();
    printf("\n");
    // Open
    printf("Opening Non-Volatile Storage (NVS) ... ");   
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
 
    if (err != ESP_OK) 
        printf("Error (%d) opening NVS!\n", err);
    else {
        printf("Done\n");       // Read
        printf("Reading string from NVS ... ");

        err = nvs_get_str(my_handle, "ssid", NULL, &required_size_ssid);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
        
        err = nvs_get_str(my_handle, "pass", NULL, &required_size_pass);        
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

        if (required_size_ssid > 0)    
			wifi_mode();			   
        else                    
			bluetooth_mode();
	}
}

