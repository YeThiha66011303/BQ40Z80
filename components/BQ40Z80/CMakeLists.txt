idf_component_register(
    SRCS "BQ40Z80.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver
    PRIV_REQUIRES esp_driver_gpio esp_driver_i2c
)