cmake_minimum_required(VERSION 3.5)

set(srcs
		"FastLEDArduinoShim.cpp"
		"FastLED/src/FastLED.cpp"
		"FastLED/src/bitswap.cpp"
		"FastLED/src/colorpalettes.cpp"
		"FastLED/src/colorutils.cpp"
		"FastLED/src/hsv2rgb.cpp"
		"FastLED/src/lib8tion.cpp"
		"FastLED/src/noise.cpp"
		"FastLED/src/platforms.cpp"
		"FastLED/src/power_mgt.cpp"
		"FastLED/src/wiring.cpp"
		"FastLED/src/platforms/esp/32/clockless_rmt_esp32.cpp"
		)

idf_component_register(SRCS "${srcs}"
			 INCLUDE_DIRS "." "FastLED/src")

set(extra_defines 
	ESP32
	# Uncomment for hardware SPI
	# FASTLED_ALL_PINS_HARDWARE_SPI
	# Uncomment tu use I2S driver
	# FASTLED_ESP32_I2S
	# Uncomment for builtin (unoptimized) RMT driver
	# FASTLED_RMT_BUILTIN_DRIVER
)
target_compile_definitions(${COMPONENT_LIB} PUBLIC ${extra_defines})

