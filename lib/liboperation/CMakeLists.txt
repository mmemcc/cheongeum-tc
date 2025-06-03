
file(GLOB SOURCES "src/*.c")

set(requires "log" "esp_common" "freertos" "driver" "esp_wifi" "nvs_flash" "console")
set(private_requires "libutil" "libsensor" "librelay" "libserver" "libtc")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include"

    REQUIRES ${requires}
    PRIV_REQUIRES ${private_requires}
)