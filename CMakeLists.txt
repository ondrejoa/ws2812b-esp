if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 freertos pthread)
else()
    set(req driver freertos pthread)
endif()

idf_component_register(
    SRCS ws2812b.cpp
    INCLUDE_DIRS .
    REQUIRES ${req}
)
