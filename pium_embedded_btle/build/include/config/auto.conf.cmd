deps_config := \
	/home/jake/pium/esp/esp-idf/components/bt/Kconfig \
	/home/jake/pium/esp/esp-idf/components/esp32/Kconfig \
	/home/jake/pium/esp/esp-idf/components/ethernet/Kconfig \
	/home/jake/pium/esp/esp-idf/components/freertos/Kconfig \
	/home/jake/pium/esp/esp-idf/components/log/Kconfig \
	/home/jake/pium/esp/esp-idf/components/lwip/Kconfig \
	/home/jake/pium/esp/esp-idf/components/mbedtls/Kconfig \
	/home/jake/pium/esp/esp-idf/components/spi_flash/Kconfig \
	/home/jake/pium/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/jake/pium/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/jake/pium/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/jake/pium/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
