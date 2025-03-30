

set(private_requires "liboperation" "libutil" "librelay" "libserver" "libtc" "esp-dsp" "esp_timer" "esp_adc")

file(GLOB SOURCES "src/*.c")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include"
    # REQUIRES ${requires}
    PRIV_REQUIRES ${private_requires}
)