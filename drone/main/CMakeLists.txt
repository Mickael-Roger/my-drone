idf_component_register(
    SRCS
        main.c
        wifi/wifi.c
        serial/serial.c
        command/command.c
    INCLUDE_DIRS
        .
        wifi
        serial
        command
)

