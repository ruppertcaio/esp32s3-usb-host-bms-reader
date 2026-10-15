idf_component_register(
    SRCS "main.cpp"
         "bms_usb_host.cpp"
         "BmsEvpvParser.cpp"
         "BmsStatusFrame.cpp"
         "report.cpp"
         "drivers/Mcp2515.cpp"
    INCLUDE_DIRS "."
    # Todo componente cujos headers são incluídos em main/
    REQUIRES usb esp_driver_spi esp_timer esp_hw_support esp_common freertos log soc
)

# O ESP-IDF v5.5 usa gnu++2b por padrão; o projeto é C++17
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)
