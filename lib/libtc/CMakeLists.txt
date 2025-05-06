
set(private_requires "liboperation" "libutil" "libsensor" "librelay" "libserver" "esp_lcd")

file(GLOB SOURCES "src/*.c")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include"
    
    # REQUIRES ${requires}
    PRIV_REQUIRES ${private_requires}
)