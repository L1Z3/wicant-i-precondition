# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.
set(srcs "main.c" "comm_server.c" "config_server.c" "realdash.c" "slcan.c" "can.c" "ble.c" "wifi_network.c" "gvret.c" "elm327.c" "mqtt.c" "sleep_mode.c" "autopid.c" "expression_parser.c" "wc_mdns.c" "wc_timer.c" "dev_status.c" "hsm.c" "precondition.c" "persistent_settings.c" "time_sync.c" "track_popup.c" "isotp_tx.c" "utf8_utf16_converter.c")
set(requires   esp_driver_twai esp_twai_mcp2515 esp_timer esp_wifi nvs_flash fatfs vfs driver esp-tls esp_adc esp_eth log app_update esp_http_server bt spiffs freertos mqtt json debug_logs ha_webhooks filesystem)

# Ensure minified HTML is generated during CMake configuration
execute_process(
    COMMAND python ${CMAKE_SOURCE_DIR}/tools/minify_html.py ${CMAKE_CURRENT_SOURCE_DIR}/homepage_full.html ${CMAKE_CURRENT_SOURCE_DIR}/homepage.html
)

idf_component_register(
    SRCS "hw_config.c" "wc_timer.c" "autopid.c" "ftp.c" "" "${srcs}"        # list the source files of this component
    INCLUDE_DIRS "."    # optional, add here public include directories
    PRIV_INCLUDE_DIRS   # optional, add here private include directories
    REQUIRES   "${requires}"         # optional, list the public requirements (component names)
    PRIV_REQUIRES       # optional, list the private requirements
    EMBED_FILES "homepage.html.gz"
)

# Also ensure it runs during `ninja` or `make` if homepage_full.html is modified without a CMake reconfigure
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/homepage.html.gz ${CMAKE_CURRENT_SOURCE_DIR}/homepage.html
    COMMAND python ${CMAKE_SOURCE_DIR}/tools/minify_html.py ${CMAKE_CURRENT_SOURCE_DIR}/homepage_full.html ${CMAKE_CURRENT_SOURCE_DIR}/homepage.html
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/homepage_full.html
    COMMENT "Minifying & Gzipping homepage_full.html -> homepage.html.gz"
)
add_custom_target(minify_html_target DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/homepage.html.gz)
add_dependencies(${COMPONENT_LIB} minify_html_target)

