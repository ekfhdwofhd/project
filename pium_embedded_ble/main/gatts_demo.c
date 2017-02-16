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
#include "driver/gpio.h"
#include "nvs.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"

#define GATTS_TAG "GATTS_DEMO"

#define OUT_1 17
#define OUT_2 18
#define OUT_3 19
#define IN_1 4
#define MINIMUM_DIFFUSING_INTERVAL_MS 500 //0.5second

static bool isDiffusing = false;

static bool newSchedule = false;
static bool cancel_Schedule = false;
uint32_t schedule_bitmap;
static char pattern1; 
static char pattern2; 
static char pattern3;
static char repeation;


///Declare the static function 
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);// 이건 지워도 될듯 

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
    },
    //~ [PROFILE_B_APP_ID] = {
        //~ .gatts_cb = gatts_profile_b_event_handler,                   /* This demo does not implement, similar as profile A */
        //~ .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    //~ },
};

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
        printf("ssssss: %s\n",param->write.value);
        
         const char *in_payload = (char*) param->write.value;
        
        
        printf("in_payload11 !:: %s\n",in_payload);
        
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);//왜 여기서 멈추나 
        
         printf("in_payload22 !:: %s\n",in_payload);
        
       
        
          if (!isDiffusing) {
    isDiffusing = true; //세마포어 같은거네 이거때문에 중간에 명령 안들어오

    char command[2]={0,};
    char repeat[3]={0,};
    char schedule[9]={0,};
    char params1[3]={0,};
    char params2[3]={0,};
    char params3[3]={0,};
    char com;
 
	strncpy(command, &in_payload[0], 1);
	com = (char)strtol(command, NULL, 16);
		
    switch(com){
		case 0xA : 
			strncpy(params1, &in_payload[1], 2);
			strncpy(params2, &in_payload[3], 2);
			strncpy(params3, &in_payload[5], 2);
		
	//	  IOT_INFO("case A !! %s %s %s %s %s", params0, command, params1, params2, params3);
		  pattern1 = (char)strtol(params1, NULL, 16);
	    pattern2 = (char)strtol(params2, NULL, 16);
	    pattern3 = (char)strtol(params3, NULL, 16);		
		  //control_led();
			newSchedule=true;
			repeation=0;
		break;
		
		case 0xB : 
			//IOT_INFO("111111111111!!!!!!!!");
			
			strncpy(repeat, &in_payload[1], 2);
			strncpy(schedule, &in_payload[3], 8);
			strncpy(params1, &in_payload[11], 2);
			strncpy(params2, &in_payload[13], 2);
			strncpy(params3, &in_payload[15], 2);
			
			//~ params1[2] = '\0';
			//~ params2[2] = '\0';
			//~ params3[2] = '\0';
			
			printf("repeat: %s\n schedule : %s\n params1: %02x%02x%02x\n params2:%s\n params3:%s\n",repeat,schedule,params1[0], params1[1], params1[2], params2,params3);
      pattern1 = (char)strtol(params1, NULL, 16);
	    pattern2 = (char)strtol(params2, NULL, 16);
	    pattern3 = (char)strtol(params3, NULL, 16);	
			
			repeation = (char)strtol(repeat, NULL, 16);
			schedule_bitmap = (uint32_t)strtoul(schedule, NULL, 16);
			//memcpy(&schedule_bitmap,schedule,strtol(schedule, NULL, 16));

			newSchedule = true;	
			//IOT_INFO("2222222222222!!!!!!!!");
		    //IOT_INFO("case B !! %s %s %s %s %s", params0, command, params1, params2, params3);
   
		break;
		
		case 0xC :
			cancel_Schedule = true;
		
		break;
		
		default: printf("Wrong command!!");
		break;
		}
		
    isDiffusing = false; //TODO: MQTT MESSAGE IS MASKED, BUT TIMER INTERRUPTS MAY OCCUR.
     printf("com:%x\n repeation:%x\n schedule_bitmap:%x\n pattern1:%x\n pattern2:%x\n pattern3:%x\n", com, repeation,schedule_bitmap, pattern1, pattern2, pattern3);
  }
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
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
  /**
   * Initialise GPIO for blinking application
   */
  gpio_pad_select_gpio(OUT_1);
  gpio_pad_select_gpio(OUT_2);
  gpio_pad_select_gpio(OUT_3);
  gpio_set_direction(OUT_1, GPIO_MODE_OUTPUT);
  gpio_set_direction(OUT_2, GPIO_MODE_OUTPUT);
  gpio_set_direction(OUT_3, GPIO_MODE_OUTPUT);
  
}

void control_led() {
  int i;
  for(i = 0; i < 8; ++i) {
    gpio_set_level(OUT_1, (pattern1>>(7-i)) & 1);
    gpio_set_level(OUT_2, (pattern2>>(7-i)) & 1);
    gpio_set_level(OUT_3, (pattern3>>(7-i)) & 1);
    vTaskDelay(MINIMUM_DIFFUSING_INTERVAL_MS / portTICK_PERIOD_MS);
  }
  gpio_set_level(OUT_1, 0);
  gpio_set_level(OUT_2, 0);
  gpio_set_level(OUT_3, 0);
}

void schedule_control()//void *pvParameters) 
{
	printf("schedule_control!!!!!!!!!!!!!!!!!!!");
	//const char pattern1 = 0xaa;
	//const char pattern2 = 0xf0;
	//const char pattern3 = 0x0f;
	//vTaskDelay(MINIMUM_DIFFUSING_INTERVAL_MS / portTICK_PERIOD_MS);//waiting 5 seconds 
	while (1){
		vTaskDelay(3000 / portTICK_PERIOD_MS);	
		if(newSchedule == true){
		//	IOT_INFO("Schedule_control!!!!!!");	
			for(int i=0; i<repeation; ++i){
				if(cancel_Schedule == true){
					cancel_Schedule = false;
					break;
				}
				if((schedule_bitmap >> (29-i) & 1) == 1){
					//IOT_INFO("%d time :::::::::: %d",i,schedule_bitmap >> (repeation-i) & 1);
					control_led();
				}
				vTaskDelay(5000 / portTICK_PERIOD_MS);//이거 한번 쉬는게 원래는 1분이여야해: 60000	
			}
			if(repeation==0){// type A defusing 
				control_led();
			}
			newSchedule = false;
		}
	}
}



/**
 *  Controls GPIO.  Patterns are received as bitmaps of 8 bits (SUBJECT TO CHANGE).
 */ //const char pattern1, const char pattern2, const char pattern3 


void app_main(){
    esp_err_t ret;
	printf("1");
    esp_bt_controller_init(); // Initialize BT controller.
	printf("2");
    ret = esp_bluedroid_init();
    	printf("3");
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed\n", __func__);
        return;
    }
    
//Enable bluetooth, must after esp_bluedroid_init()
//~ Return
//~ ESP_OK : Succeed
//~ Other : Failed
	printf("4\n");
    ret = esp_bluedroid_enable(); 
    	printf("5\n");
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed\n", __func__);
        return;
    }
		printf("6\n");
    esp_ble_gatts_register_callback(gatts_event_handler);
    	printf("7\n");
    esp_ble_gap_register_callback(gap_event_handler);
    	printf("8\n");
   // esp_ble_gatts_app_register(PROFILE_B_APP_ID);

	initialise_gpio();
	printf("9\n");
	//schedule_control();
		printf("10\n");
	 xTaskCreate(&schedule_control, "Schedule Control", 8192*2, NULL, 5, NULL);//, 1);
	printf("11\n");
   esp_ble_gatts_app_register(PROFILE_A_APP_ID);
 
    return;
}
