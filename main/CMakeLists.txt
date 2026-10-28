file(GLOB_RECURSE SOURCES_C *.c)
file(GLOB_RECURSE SOURCES_CPP *.cpp)

# ---- UI theme table codegen ------------------------------------------------
# themes/registry.txt + themes/*/<id>/theme.toml -> export_path/ui_theme_generated.c
# Runs BEFORE idf_component_register because SRC_DIRS is globbed at register time.
# Adding a theme requires editing registry.txt, and registry.txt is watched
# below, so a new theme always triggers a reconfigure and regeneration.
set(THEMES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../themes")
file(GLOB THEME_MANIFESTS "${THEMES_DIR}/*/*/theme.toml")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${THEMES_DIR}/registry.txt" ${THEME_MANIFESTS})

# idf.py sets `python` to the IDF virtualenv interpreter; fall back for plain
# cmake / IDE reconfigures. gen_themes.py has no third-party dependencies.
if(NOT DEFINED python)
    set(python "python3")
endif()

execute_process(
    COMMAND "${python}" "${CMAKE_CURRENT_SOURCE_DIR}/../tools/gen_themes.py"
    RESULT_VARIABLE THEME_GEN_RESULT
    OUTPUT_VARIABLE THEME_GEN_STDOUT
    ERROR_VARIABLE  THEME_GEN_STDERR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE)
if(NOT THEME_GEN_RESULT EQUAL 0)
    message(FATAL_ERROR
            "Theme generation failed.\n${THEME_GEN_STDOUT}\n${THEME_GEN_STDERR}\n"
            "See themes/README.md.")
endif()
if(THEME_GEN_STDERR)
    message(WARNING "${THEME_GEN_STDERR}")
endif()
message(STATUS "${THEME_GEN_STDOUT}")

# Build metadata for app-side compatibility checks and on-device manifests.
set(OBD_GAUGE_GIT_HASH "unknown")
set(OBD_GAUGE_GIT_BRANCH "unknown")
set(OBD_GAUGE_GIT_COUNT 0)

execute_process(
    COMMAND git -C "${CMAKE_CURRENT_SOURCE_DIR}/.." rev-parse --short=12 HEAD
    RESULT_VARIABLE OBD_GAUGE_GIT_HASH_RESULT
    OUTPUT_VARIABLE OBD_GAUGE_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT OBD_GAUGE_GIT_HASH_RESULT EQUAL 0 OR OBD_GAUGE_GIT_HASH STREQUAL "")
    set(OBD_GAUGE_GIT_HASH "unknown")
endif()

execute_process(
    COMMAND git -C "${CMAKE_CURRENT_SOURCE_DIR}/.." rev-parse --abbrev-ref HEAD
    RESULT_VARIABLE OBD_GAUGE_GIT_BRANCH_RESULT
    OUTPUT_VARIABLE OBD_GAUGE_GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT OBD_GAUGE_GIT_BRANCH_RESULT EQUAL 0 OR OBD_GAUGE_GIT_BRANCH STREQUAL "")
    set(OBD_GAUGE_GIT_BRANCH "unknown")
endif()

execute_process(
    COMMAND git -C "${CMAKE_CURRENT_SOURCE_DIR}/.." rev-list --count HEAD
    RESULT_VARIABLE OBD_GAUGE_GIT_COUNT_RESULT
    OUTPUT_VARIABLE OBD_GAUGE_GIT_COUNT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT OBD_GAUGE_GIT_COUNT_RESULT EQUAL 0 OR OBD_GAUGE_GIT_COUNT STREQUAL "")
    set(OBD_GAUGE_GIT_COUNT 0)
endif()

string(REGEX REPLACE "[^A-Za-z0-9._-]" "_" OBD_GAUGE_GIT_BRANCH_SAFE "${OBD_GAUGE_GIT_BRANCH}")
set(OBD_GAUGE_BUILD_TAG "${OBD_GAUGE_GIT_BRANCH_SAFE}-${OBD_GAUGE_GIT_COUNT}-${OBD_GAUGE_GIT_HASH}")

idf_component_register(SRC_DIRS
                    "."
                    "export_path/components"
                    "export_path/fonts"
                    "export_path/images"
                    "export_path/theme_assets"
                    "export_path/screens"
                    "export_path"
                    "app_obd_dsp"
                    "bsp_obd_dsp"
                    "bsp_obd_dsp/lcd_driver"
                    "bsp_obd_dsp/lcd_driver/esp_lcd_st77916"
                    "bsp_obd_dsp/exio"
                    "bsp_obd_dsp/i2c_driver"
                    "bsp_obd_dsp/touch_driver"
                    "theme_engine"
                    INCLUDE_DIRS
                    "."
                    REQUIRES
                    lvgl__lvgl
                    esp_http_server
                    esp_wifi
                    bt
                    spiffs
                    mbedtls
                    nvs_flash
                    app_update
                    json)

# Auto-generate the bootmedia SPIFFS image; flashed together via idf.py flash
spiffs_create_partition_image(bootmedia ${CMAKE_CURRENT_SOURCE_DIR}/../bootmedia/slot_a FLASH_IN_PROJECT)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    OBD_GAUGE_GIT_HASH="${OBD_GAUGE_GIT_HASH}"
    OBD_GAUGE_GIT_BRANCH="${OBD_GAUGE_GIT_BRANCH_SAFE}"
    OBD_GAUGE_GIT_COUNT=${OBD_GAUGE_GIT_COUNT}
    OBD_GAUGE_BUILD_TAG="${OBD_GAUGE_BUILD_TAG}")

# Only LVGL uses explicit PSRAM; shared definitions ensure identical config.
# This header contains inline hooks, so no main<->LVGL linker dependency.
idf_component_get_property(gauge_lvgl_lib lvgl__lvgl COMPONENT_LIB)
target_include_directories(${gauge_lvgl_lib} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(${gauge_lvgl_lib} PUBLIC
    LV_MEM_CUSTOM_INCLUDE="gauge_lvgl_memory.h"
    LV_MEM_CUSTOM_ALLOC=gauge_lvgl_alloc
    LV_MEM_CUSTOM_FREE=gauge_lvgl_free
    LV_MEM_CUSTOM_REALLOC=gauge_lvgl_realloc)

# Local immutable SDK identity layer; global SDK stays unchanged.
include("${CMAKE_CURRENT_LIST_DIR}/../private_bt/replace_bt_sources.cmake")

# Preserve the SDK source-specific flags on the two private replacement paths.
set_source_files_properties(
    "${gauge_bt_root}/components/bt/host/bluedroid/bta/gatt/bta_gattc_act.c"
    "${gauge_bt_root}/components/bt/host/bluedroid/bta/gatt/bta_gattc_cache.c"
    TARGET_DIRECTORY "${gauge_bt_lib}"
    PROPERTIES COMPILE_FLAGS "-Wno-address-of-packed-member")
