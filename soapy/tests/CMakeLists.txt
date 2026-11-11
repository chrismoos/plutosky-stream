set(PLUTOSKY_TESTS
    test_json
    test_integer_hz
    test_formats
    test_vita
    test_convert
    test_time
    test_serial_alias
    test_hardware_key
    test_rx_time_offset
    test_status_log
    test_tx_pacer)

foreach(name ${PLUTOSKY_TESTS})
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE plutosky_core)
    target_compile_definitions(${name} PRIVATE
        PLUTOSKY_VECTORS_DIR="${PLUTOSKY_VECTORS_DIR}")
    add_test(NAME ${name} COMMAND ${name})
endforeach()

target_link_libraries(test_time PRIVATE SoapySDR)
