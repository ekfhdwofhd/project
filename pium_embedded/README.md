# Pium Embedded README

## ESP32 & AWS IoT Integration

### How to build & flash
Default BAUD rate is set to 115200.  Silicon Labs' USB to UART driver is necessary.

At the root directory, type in the following:
```
make flash
```

To only flash the application partition (excluding bootloader), you can do:
```
make app-flash
```

### Following files must be modified to suit environment: 
```
main/main.c
main/include/aws_iot_config.h
main/certs
```

####main/main.c
WiFi connection information is statically provided in this file.
Please modify the following two lines:
```
#define WIFI_SSID "(your_wifi_ssid)"
#define WIFI_PASS "(your_wifi_pass)"
```

####main/include/aws_iot_config.h
Three lines in the file must be modified to fit your Thing in AWS IoT.  Information is found in your AWS IoT Console.
```
#define AWS_IOT_MQTT_HOST         "(unique_id).iot.(region).amazonaws.com"
#define AWS_IOT_MQTT_CLIENT_ID    "(your_client_id)"
#define AWS_IOT_MY_THING_NAME     "(your_thing_name)"
```

####main/certs
This directory should contain valid certificate files.  The names of the files must match the following:
```
root-CA.crt
certificate.pem.crt
private.pem.key
```
Refer to [Downloads](https://bitbucket.org/pium/pium_embedded/downloads)

## Wiki Link
[Wiki](https://bitbucket.org/pium/pium_embedded/wiki/Home)