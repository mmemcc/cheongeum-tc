
set(private_requires "liboperation" "libutil" "libsensor" "librelay" "libtc" "esp_http_client" "esp-tls" "mqtt" "json")

file(GLOB SOURCES "src/*.c")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include"

    # REQUIRES ${requires} 
    PRIV_REQUIRES ${private_requires}
)