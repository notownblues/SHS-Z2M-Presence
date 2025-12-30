idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."
    REQUIRES ld2410_enhanced ld2450 light_driver zcl_utility
    PRIV_REQUIRES nvs_flash esp-zigbee-lib driver esp_wifi esp_http_server esp_netif
)
