idf_component_register(
    SRCS "df_settings.cpp" "main.cpp"
         "telemetry.cpp"
         "telemetry_json.cpp"
         "wifi_ap.cpp"
         "http_server.cpp"
         "ble_server.cpp"
         "df_settings.cpp"
    INCLUDE_DIRS "."
    REQUIRES BQ40Z80 esp_wifi esp_netif esp_event nvs_flash esp_http_server esp_timer bt
    EMBED_TXTFILES "www/index.html"
)
