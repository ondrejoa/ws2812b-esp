COMPONENT_ADD_INCLUDEDIRS = .

ifdef CONFIG_IDF_TARGET_ESP8266
COMPONENT_DEPENDS = esp8266 freertos pthread
else
COMPONENT_DEPENDS = driver freertos pthread
endif
