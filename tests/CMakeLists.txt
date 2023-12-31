## ./tests/CMakeLists.txt
set(tftTests TRUE)
if (${_MSDK_BOARD_NAME_} MATCHES "(^CLICKER(.+)$|^(.+)CLICKER$|^(.+)CURIOSITY(.+)$|^FLIP_AND_CLICK_PIC32MZ$|^GENERIC_BOARD$|^BOARD_PICPLC16_V7$|^(.+)_V7_(.+)$|^(.+)V7$|^BOARD_FLOWPAW$|^BOARD_QUAIL$|^BOARD_MICROMOD(.+))")
    set(tftTests FALSE)
endif()

set(clickTests TRUE)
if (${_MSDK_BOARD_NAME_} MATCHES "(^GENERIC_BOARD$)")
    set(clickTests FALSE)
endif()

set(tftJigTest TRUE)
if (${_MSDK_BOARD_NAME_} MATCHES "^MIKROMEDIA(.+)$")
    set(tftJigTest FALSE)
endif()

add_subdirectory(gpio)

## 8K Flash and 2K RAM minimum for this test.
memory_sdk_test_check(memory_issue 8192 2048)

if (${memory_issue} STREQUAL "FALSE")
    add_subdirectory(adc)
else()
    message(INFO ": ${MCU_NAME} does not have enough memory for \"adc\" test.")
endif()

## 8K Flash and 2K RAM minimum for this test.
memory_sdk_test_check(memory_issue 8192 2048)

if (${memory_issue} STREQUAL "FALSE")
    add_subdirectory(i2c)
else()
    message(INFO ": ${MCU_NAME} does not have enough memory for \"i2c\" test.")
endif()

## 8K Flash and 4K RAM minimum for this test.
memory_sdk_test_check(memory_issue 8192 4096)

if (${memory_issue} STREQUAL "FALSE")
    add_subdirectory(pwm)
else()
    message(INFO ": ${MCU_NAME} does not have enough memory for \"pwm\" test.")
endif()

## 8K Flash and 2K RAM minimum for this test.
memory_sdk_test_check(memory_issue 8192 2048)

if (${memory_issue} STREQUAL "FALSE")
    add_subdirectory(spi)
else()
    message(INFO ": ${MCU_NAME} does not have enough memory for \"spi\" test.")
endif()

## 8K Flash and 2K RAM minimum for this test.
memory_sdk_test_check(memory_issue 8192 2048)

if (${memory_issue} STREQUAL "FALSE")
    add_subdirectory(uart)
else()
    message(INFO ": ${MCU_NAME} does not have enough memory for \"uart\" test.")
endif()

if (_HAS_MIKROBUS_)
    if (${_HAS_MIKROBUS_} STREQUAL TRUE)
        add_subdirectory(mikrobus)
    endif()
endif()

if (${_MSDK_BOARD_NAME_} MATCHES "CLICKER")
    add_subdirectory(clicker_buttons)
endif()

add_subdirectory(one_wire)

if (_MSDK_TFT_BOARD_)
    find_package(MikroSDK.GraphicLibrary)
    if (MikroSDK.GraphicLibrary\_FOUND)
        find_package(Components.TftBoard8)
        if (Components.TftBoard8\_FOUND)
            add_subdirectory(gl)
        endif()
    endif()
endif()

if (clickTests)
    add_subdirectory(clicks)
endif()
if (tftTests)
    if(tftJigTest)
        add_subdirectory(tft_jig)
    endif()
endif()

add_subdirectory(rtc)