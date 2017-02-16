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
#include "esp_log.h"
#include "nvs_flash.h"
#include "bt.h"
#include "bta_api.h"

#include "esp_attr.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_smartconfig.h"
#include "nvs.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "timer_interface.h"

#include "esp_wifi.h"

#define GATTS_TAG "GATTS_DEMO"

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

#define ONE_MINUTE 60

static bool newSchedule = false;
static bool cancelSchedule = false;
static bool requestSchedule = false;
uint32_t schedule_bitmap;
static char pattern1; 
static char pattern2; 
static char pattern3;
static char scheduleDuration;


///Declare the static function 
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

#define GATTS_SERVICE_UUID_TEST_B   0x00EE
#define GATTS_CHAR_UUID_TEST_B      0xEE01
#define GATTS_DESCR_UUID_TEST_B     0x2222
#define GATTS_NUM_HANDLE_TEST_B     4

#define TEST_DEVICE_NAME            "ESP_GATTS_DEMO"
#define TEST_MANUFACTURER_DATA_LEN  17
static uint8_t test_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAB, 0xCD, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAB, 0xCD, 0xAB, 0xCD,
};

//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
static esp_ble_adv_data_t test_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = test_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t test_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
//#define PROFILE_B_APP_ID 1

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    }
};

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
static bool gl_sta_connected = false;
static uint8_t gl_sta_bssid[6];
static uint8_t gl_sta_ssid[32];
static int gl_sta_ssid_len;

static wifi_config_t sta_config;
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
//~ #define WIFI_SSID "CREATIVE@33-2.4"
//~ #define WIFI_PASS "creative44"
/////////////////////////////////////////////////////aws end


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
 




static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&test_adv_params);
        break;
    default:
        break;
    }
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        esp_ble_gap_config_adv_data(&test_adv_data);

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %16x\n", param->write.len, *(uint32_t *)param->write.value);
        
        const char *in_payload = (char*) param->write.value;
        
        ESP_LOGI(GATTS_TAG, "Text received: %s\n",param->write.value);
       
       // char *token = NULL;
         
		//char str1[] = "This is\tTest,Program!\n";
	//	char token_separation[] = " ,\t\n";//' '이나 ','이나 '\t'이나 '\n'이 들어있는 문자열을 분리
	 
	//	token = strtok( in_payload, token_separation );
	 
		printf("param: %d , param->write %d ,param->write.value: %d\n",sizeof(param),sizeof(param->write),sizeof(param->write.value));
		
		
       //~ esp_wifi_set_config(WIFI_IF_STA, &sta_config);  
      wifi_config_t wifi_config;
       //~ wifi_config_t wifi_config = {
        //~ .sta = {
        //~ .ssid ="",//= WIFI_SSID, 
        //~ .password =""//= WIFI_PASS,
        //~ },
    //~ };
    
    
    
    
    
    
   
		
          
          
          char store_option[2]={0,};
          char op_parsing;
          strncpy(store_option, &in_payload[0], 1);
          op_parsing = (char)strtol(store_option, NULL, 16);
          switch(op_parsing){
			  case 0xA : 
			         strncpy((char *)wifi_config.sta.ssid, &in_payload[1], param->write.len);
			         //strcpy((char *)wifi_config.sta.ssid, (char *)in_payload);
			//~ op_parsing = (char)strtol(store_option, NULL, 16);
			//strcpy((char *)wifi_config.sta.ssid, (char *)in_payload);
						
						     
    
			  err = nvs_set_str(my_handle, "ssid", (char *)wifi_config.sta.ssid); //nvs setting.
			
					printf( "ssid = %s\n", wifi_config.sta.ssid );
			  		printf( "passwd = %s\n", wifi_config.sta.password );	
			  		//~ ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
					//~ ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
					//~ ESP_ERROR_CHECK( esp_wifi_start() ); 
			  break;
			  
			  case 0xB :
				memset(&wifi_config,0,sizeof(wifi_config));
				
				  char* my_ssid = malloc(required_size_ssid);
				 err = nvs_get_str(my_handle, "ssid", my_ssid, &required_size_ssid);//get ssid
				  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
					strcpy((char *)wifi_config.sta.ssid, (char *)my_ssid); 
				   strncpy((char *)wifi_config.sta.password, &in_payload[1], param->write.len);
				//  strcpy((char *)wifi_config.sta.password, (char *)in_payload);
		
					//~ err = nvs_set_str(my_handle, "pass", (char *)wifi_config.sta.password);
		
					nvs_commit(my_handle);//Write any pending changes to non-volatile storage
					nvs_close(my_handle);
					printf( "ssid = %s\n", wifi_config.sta.ssid );
			  		printf( "passwd = %s\n", wifi_config.sta.password );
   // fprintf(stderr, "Setting WiFi configuration SSID %s...\n", wifi_config.sta.ssid);
					ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
					ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
					ESP_ERROR_CHECK( esp_wifi_start() ); 
			  break;
			  
			  default:
				printf("Wrong option!");
			  break;
		  }
			  
			  //~ }
          //~ if(store_option=="1"){
			        //~ err = nvs_set_str(my_handle, "ssid", (char *)wifi_config.sta.ssid); //nvs setting.
			
					//~ printf( "ssid = %s\n", wifi_config.sta.ssid );
			  		//~ printf( "passwd = %s\n", wifi_config.sta.password );			  }
			  
		  //~ else{
					//~ err = nvs_set_str(my_handle, "pass", (char *)wifi_config.sta.password);
		
					//~ nvs_commit(my_handle);//Write any pending changes to non-volatile storage
					//~ nvs_close(my_handle);
					//~ printf( "ssid = %s\n", wifi_config.sta.ssid );
			  		//~ printf( "passwd = %s\n", wifi_config.sta.password );
   //~ // fprintf(stderr, "Setting WiFi configuration SSID %s...\n", wifi_config.sta.ssid);
					//~ ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
					//~ ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
					//~ ESP_ERROR_CHECK( esp_wifi_start() ); 
			  //~ }
          
    
       
        //#define WIFI_PASS "creative44"
		//strcpy((char *)sta_config.sta.password, "creative44");
       
        //~ token = strtok( NULL, token_separation );
    
        //~ strcpy((char *)sta_config.sta.password, (char *)token);


       
       
        //~ char command[2]={0,};
        //~ char repeat[3]={0,};
        //~ char schedule[9]={0,};
        //~ char params1[3]={0,};
        //~ char params2[3]={0,};
        //~ char params3[3]={0,};
        //~ char com;

        //~ strncpy(command, &in_payload[0], 1);
        //~ com = (char)strtol(command, NULL, 16);
        
        //~ switch (com) {
            //~ case 0xA : 
                //~ strncpy(params1, &in_payload[1], 2);
                //~ strncpy(params2, &in_payload[3], 2);
                //~ strncpy(params3, &in_payload[5], 2);
                
                //~ pattern1 = (char)strtol(params1, NULL, 16);
                //~ pattern2 = (char)strtol(params2, NULL, 16);
                //~ pattern3 = (char)strtol(params3, NULL, 16);

                //~ newSchedule=true;
                //~ scheduleDuration=0;
                //~ ESP_LOGI(GATTS_TAG, "Command A:\nFlaskA: %s\nFlaskB: %s\nFlaskC: %s\n", params1, params2, params3);
                //~ break;
            
            //~ case 0xB :       
                //~ strncpy(repeat, &in_payload[1], 2);
                //~ strncpy(schedule, &in_payload[3], 8);
                //~ strncpy(params1, &in_payload[11], 2);
                //~ strncpy(params2, &in_payload[13], 2);
                //~ strncpy(params3, &in_payload[15], 2);

                //~ pattern1 = (char)strtol(params1, NULL, 16);
                //~ pattern2 = (char)strtol(params2, NULL, 16);
                //~ pattern3 = (char)strtol(params3, NULL, 16);	
                
                //~ scheduleDuration = (char)strtol(repeat, NULL, 10);
                //~ schedule_bitmap = (uint32_t)strtoul(schedule, NULL, 16);

                //~ newSchedule = true;	
                //~ ESP_LOGI(GATTS_TAG, "Command B:\nSchedule Duration: %d\nSchedule Pattern:0x%X\nFlaskA: %s\nFlaskB: %s\nFlaskC: %s\n", scheduleDuration, schedule_bitmap, params1, params2, params3);
                //~ break;

            //~ case 0xC :
                //~ cancelSchedule = true;
                //~ ESP_LOGI(GATTS_TAG, "Command C:\nCanel Current Schedule");
                //~ break;
            
            //~ default: 
                //~ ESP_LOGI(GATTS_TAG, "Wrong command!!");
                //~ break;
        //~ }
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
    case ESP_GATTS_MTU_EVT:
    case ESP_GATTS_CONF_EVT:
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);

        esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,NULL,NULL);
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,NULL,NULL);
        break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, is_conn %d\n",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5],
                 param->connect.is_connected);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        break;
    case ESP_GATTS_DISCONNECT_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static esp_err_t event_handler_blemode(void *ctx, system_event_t *event)
{	
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP://wifi connent
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
             //~ err = nvs_set_str(my_handle, "ssid", (char *)sta_config.sta.ssid); //nvs setting.
			 //~ err = nvs_set_str(my_handle, "pass", (char *)sta_config.sta.password);
			
			  //~ nvs_commit(my_handle);//Write any pending changes to non-volatile storage
			  //~ nvs_close(my_handle);
			  //~ fflush(stdout);
				//~ if(required_size_ssid==0){
				 //~ for (int i = 5; i >= 0; i--) {
					//~ printf("Restarting in %d seconds...\n", i);
					//~ vTaskDelay(1000 / portTICK_RATE_MS);
				//~ }
					//~ esp_restart();//reboot.
				//~ }    
				 printf("connect!!!!!\n");
				 //xTaskCreate(&aws_iot_mqtt_task, "AWS IoT", 8192*2, NULL, 5, NULL);
				break;
        case SYSTEM_EVENT_STA_DISCONNECTED://wifi disconnect			
			printf("disconnect!!!!!!!!!!!!!!!!!\n");
								
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
    //~ ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    //~ ESP_ERROR_CHECK( esp_wifi_start() );
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param){
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id, 
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
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

void control_led() {
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



/**
 *  Controls GPIO.  Patterns are received as bitmaps of 8 bits (SUBJECT TO CHANGE).
 */ //const char pattern1, const char pattern2, const char pattern3 


void app_main(){
    esp_err_t ret;
    
    
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
	}
    
    
    
	initialise_wifi_blemode();
    esp_bt_controller_init(); // Initialize BT controller.
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed\n", __func__);
        return;
    }
    ret = esp_bluedroid_enable(); 
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed\n", __func__);
        return;
    }
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);

    initialise_gpio();
    xTaskCreate(&schedule_control, "Schedule Control", 512, NULL, 5, NULL);

    esp_ble_gatts_app_register(PROFILE_A_APP_ID);
 
    return;
}
