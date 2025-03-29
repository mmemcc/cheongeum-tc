set(private_requires "liboperation" "libutil" "libsensor" "libserver" "libtc" "librelay")

file(GLOB SOURCES "*.c")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "."
    # REQUIRES ${requires}
    PRIV_REQUIRES ${private_requires}
)