if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 freertos)
else()
    set(req driver freertos)
endif()

idf_component_register(
    SRCS ws2812b.cpp
    INCLUDE_DIRS .
    REQUIRES ${req}
)
