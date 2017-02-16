
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
//#include <time.h>
//#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_attr.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "certs.h"
#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

//TODO: SHOULD BE ABLE TO DYNAMICALLY CONNECT TO WIFI
#define WIFI_SSID "CREATIVE@33-2.4"
#define WIFI_PASS "creative44"

//MQTT PUB/SUB RELATED CONSTANTS
#define TOPIC_PUBLISH "toServer"

#define PUBLISH_INTERVAL_SEC 150
#define YIELD_INTERVAL_MS 4000
#define ONE_MINUTE 60

#define STORAGE_NAMESPACE "storage"

//GPIO-RELATED CONSTANTS

#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define LEDC_IO_0   17
#define LEDC_IO_1   18
#define LEDC_IO_2   19

#define IN_1 4
#define MINIMUM_DIFFUSING_INTERVAL_MS 500 //0.5second


static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static bool newSchedule = false;
static bool cancelSchedule = false;
static bool requestSchedule = false;
uint32_t schedule_bitmap;
static char pattern1; 
static char pattern2; 
static char pattern3;
static char scheduleDuration;

/**
 *  Callback upon being disconnected from MQTT connection with the server
 *  (TODO: find out what is in 'void *pdata' parameter).
 */
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

/**
 *  Controls GPIO.  Patterns are received as bitmaps of 8 bits (SUBJECT TO CHANGE).
 */ //const char pattern1, const char pattern2, const char pattern3 

static void control_led(){
  // int i;
 // printf("control_led start!!!!!!\n");
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
/**
 *  Upon receiving message, this callback is called. 
 *  Message can be read from 'params->payload'.
 */
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
        
        default: IOT_INFO("Wrong command!!");
            break;
    }
}
    
/**
 *  DO NOT MODIFY.
 *    - DEFAULT SETTING FOR ALL ESP32 WiFi handler.
 */
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
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

/**
 *  TODO: If memory has stored value for WiFi SSID and PASSWD, load and use that settings. 
 *        If not, (either boot in SoftAP mode or enable Bluetooth Stack and receive config) 
 */
static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
        .ssid = WIFI_SSID, 
        .password = WIFI_PASS,
        },
    };
    fprintf(stderr, "Setting WiFi configuration SSID %s...\n", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() ); 
}
/**
 * Initialise GPIO for blinking application
 */

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

void app_main(void)
{
    initialise_wifi();
    initialise_gpio();
    xTaskCreate(&aws_iot_mqtt_task, "AWS IoT", 8192*2, NULL, 5, NULL);
    xTaskCreate(&schedule_control, "Scheduler", 512, NULL, 5, NULL);
}
