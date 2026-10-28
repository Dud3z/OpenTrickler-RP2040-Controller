/*
 * LED blink with FreeRTOS
 */

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "FreeRTOSConfig.h"
#include "configuration.h"
#include "u8g2.h"

// modules
#include "app.h"
#include "motors.h"
#include "eeprom.h"
#include "scale.h"
#include "display.h"
#include "charge_mode.h"
#include "rest_endpoints.h"
#include "wireless.h"
#include "neopixel_led.h"
#include "mini_12864_module.h"
#include "menu.h"
#include "profile.h"
#include "servo_gate.h"
#include "ai_trickle.h"
#include "recipe.h"
#include "feature_config.h"
#include "system_monitor.h"


// Show the init step on the display, so a hang during start up can be located
static void boot_status(const char * step) {
    u8g2_t * display = get_display_handler();
    u8g2_ClearBuffer(display);
    u8g2_SetFont(display, u8g2_font_helvB08_tr);
    u8g2_DrawStr(display, 5, 10, "OpenTrickler");
    u8g2_DrawHLine(display, 0, 13, u8g2_GetDisplayWidth(display));
    u8g2_SetFont(display, u8g2_font_helvR08_tr);
    u8g2_DrawStr(display, 5, 30, "Starting:");
    u8g2_DrawStr(display, 5, 42, step);
    u8g2_SendBuffer(display);
}


int main()
{
    // stdio_init_all();
    // Initialize EEPROM first
    eeprom_init();

    // Why did the trickler restart last time (error log), then the switches
    // of the optional functions (used by the modules below)
    system_monitor_init();
    feature_config_init();

    // Initialize Neopixel RGB on the mini 12864 board
    neopixel_led_init();

    // Configure other functions from mini 12864 display
    mini_12864_module_init();

    // Initialize wireless settings
    boot_status("Wireless");
    wireless_init();

    // Load config for motors
    boot_status("Motors");
    motor_init_err_t motor_init_err = motors_init();
    if (motor_init_err != MOTOR_INIT_OK) {
        handle_motor_init_error(motor_init_err);
    }

    // Initialize UART
    boot_status("Scale");
    scale_init();

    // Initialize charge mode settings
    boot_status("Charge mode");
    charge_mode_config_init();

    // Initialize profile data
    boot_status("Profiles");
    profile_data_init();

    // Initialize the adaptive trickle models (after the profiles)
    boot_status("AI Trickle");
    ai_trickle_init();

    // Recipes (after the profiles)
    boot_status("Recipes");
    recipe_init();

    // Initialize the servo
    boot_status("Servo gate");
    servo_gate_init();

    // Start menu task
    boot_status("Menu");
    xTaskCreate(menu_task, "Menu Task", 1024, NULL, 6, NULL);

    // Watchdog and housekeeping (dimming)
    system_monitor_start();

    // Start RTOS
    vTaskStartScheduler();

    while (true)
        ;
}
