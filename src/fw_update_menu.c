/*
 * Firmware update from the display (Settings - Firmware Update): look for a
 * new build on GitHub, download and verify it, restart into it. Pico 2 W only;
 * the work happens in fw_update.c, this is the screen and the knob.
 */
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <u8g2.h>

#include "fw_update.h"
#include "display.h"
#include "mini_12864_module.h"

#define FW_MENU_CHECK_TIMEOUT_MS        45000
#define FW_MENU_DOWNLOAD_TIMEOUT_MS     300000
#define FW_MENU_RETURN_FORM             30      // Settings


static void draw_title(u8g2_t * u8g2, const char * title) {
    u8g2_SetFont(u8g2, u8g2_font_helvB08_tr);
    u8g2_DrawStr(u8g2, 5, 10, title);
    u8g2_DrawHLine(u8g2, 0, 13, u8g2_GetDisplayWidth(u8g2));
    u8g2_SetFont(u8g2, u8g2_font_helvR08_tr);
}


// Text over up to three lines (the errors are short sentences)
static void draw_wrapped(u8g2_t * u8g2, int y, const char * text) {
    char line[32];
    const char * p = text;
    for (int row = 0; row < 3 && *p; row += 1) {
        size_t take = strlen(p);
        if (take > sizeof(line) - 1) {
            take = sizeof(line) - 1;
        }
        while (true) {
            memcpy(line, p, take);
            line[take] = '\0';
            if (take <= 1 || u8g2_GetStrWidth(u8g2, line) <= 118) {
                break;
            }
            // Shorter: back to the previous space, or one character less
            size_t cut = take - 1;
            while (cut > 0 && p[cut] != ' ') {
                cut -= 1;
            }
            take = cut > 0 ? cut : take - 1;
        }
        u8g2_DrawStr(u8g2, 5, y + row * 11, line);
        p += take;
        while (*p == ' ') {
            p += 1;
        }
    }
}


static void draw_screen(u8g2_t * u8g2, const fw_download_status_t * st) {
    char buf[40];
    u8g2_ClearBuffer(u8g2);
    draw_title(u8g2, "Firmware Update");

    switch (st->state) {
        case FW_DOWNLOAD_IDLE:
        case FW_DOWNLOAD_CHECKING:
            snprintf(buf, sizeof(buf), "Installed: build %d", st->running_build);
            u8g2_DrawStr(u8g2, 5, 26, buf);
            u8g2_DrawStr(u8g2, 5, 40, "Looking on GitHub...");
            break;

        case FW_DOWNLOAD_CHECKED:
            snprintf(buf, sizeof(buf), "Installed: build %d", st->running_build);
            u8g2_DrawStr(u8g2, 5, 26, buf);
            snprintf(buf, sizeof(buf), "Available: build %d", st->remote_build);
            u8g2_DrawStr(u8g2, 5, 37, buf);
            if (st->remote_build > st->running_build) {
                u8g2_DrawStr(u8g2, 5, 50, "Press: install");
            }
            else {
                u8g2_DrawStr(u8g2, 5, 50, "Up to date");
            }
            u8g2_DrawStr(u8g2, 5, 62, "Back: other button");
            break;

        case FW_DOWNLOAD_DOWNLOADING: {
            snprintf(buf, sizeof(buf), "Installing build %d", st->remote_build);
            u8g2_DrawStr(u8g2, 5, 26, buf);
            uint32_t pct = st->size ? (uint32_t) ((uint64_t) st->received * 100 / st->size) : 0;
            u8g2_DrawFrame(u8g2, 5, 32, 118, 10);
            u8g2_DrawBox(u8g2, 7, 34, (114 * pct) / 100, 6);
            snprintf(buf, sizeof(buf), "%lu %%  (%lu kB)", (unsigned long) pct, (unsigned long) (st->received / 1024));
            u8g2_DrawStr(u8g2, 5, 54, buf);
            u8g2_DrawStr(u8g2, 5, 63, "Do not switch off");
            break;
        }

        case FW_DOWNLOAD_DONE:
            snprintf(buf, sizeof(buf), "Build %d verified", st->remote_build);
            u8g2_DrawStr(u8g2, 5, 26, buf);
            u8g2_DrawStr(u8g2, 5, 40, "Press: restart and");
            u8g2_DrawStr(u8g2, 5, 51, "start the new firmware");
            u8g2_DrawStr(u8g2, 5, 62, "Back: later (web page)");
            break;

        case FW_DOWNLOAD_ERROR:
        default:
            u8g2_DrawStr(u8g2, 5, 26, "Not possible:");
            draw_wrapped(u8g2, 38, st->error && *st->error ? st->error : "Unknown error");
            break;
    }
    u8g2_SendBuffer(u8g2);
}


uint8_t fw_update_display_menu(void) {
    u8g2_t * u8g2 = get_display_handler();
    fw_download_status_t st;

    if (!fw_download_supported()) {
        u8g2_ClearBuffer(u8g2);
        draw_title(u8g2, "Firmware Update");
        draw_wrapped(u8g2, 26, "Updates over WiFi need the Pico 2 W. On the Pico W copy the UF2 file over USB.");
        u8g2_DrawStr(u8g2, 5, 62, "Press to go back");
        u8g2_SendBuffer(u8g2);
        while (true) {
            ButtonEncoderEvent_t event = button_wait_for_input(true);
            if (event == BUTTON_RST_PRESSED || event == BUTTON_ENCODER_PRESSED) {
                return FW_MENU_RETURN_FORM;
            }
        }
    }

    fw_download_check();
    TickType_t started = xTaskGetTickCount();

    while (true) {
        fw_download_status(&st);

        // A request without any answer is given up
        if (st.state == FW_DOWNLOAD_CHECKING && xTaskGetTickCount() - started > pdMS_TO_TICKS(FW_MENU_CHECK_TIMEOUT_MS)) {
            fw_download_abandon("No answer from GitHub (internet?)");
            continue;
        }
        if (st.state == FW_DOWNLOAD_DOWNLOADING && xTaskGetTickCount() - started > pdMS_TO_TICKS(FW_MENU_DOWNLOAD_TIMEOUT_MS)) {
            fw_download_abandon("Download took too long");
            continue;
        }

        draw_screen(u8g2, &st);

        ButtonEncoderEvent_t event = button_wait_for_input(false);
        if (event == BUTTON_RST_PRESSED) {
            // Leaving during the download would leave half an image: stay
            if (st.state != FW_DOWNLOAD_DOWNLOADING) {
                return FW_MENU_RETURN_FORM;
            }
        }
        else if (event == BUTTON_ENCODER_PRESSED) {
            if (st.state == FW_DOWNLOAD_CHECKED && st.remote_build > st.running_build) {
                if (fw_download_install()) {
                    started = xTaskGetTickCount();
                }
            }
            else if (st.state == FW_DOWNLOAD_DONE) {
                u8g2_ClearBuffer(u8g2);
                draw_title(u8g2, "Firmware Update");
                u8g2_DrawStr(u8g2, 5, 30, "Restarting...");
                u8g2_SendBuffer(u8g2);
                if (!fw_update_apply()) {
                    // Shown with the next screen
                    fw_download_abandon("Restart not possible now");
                }
            }
            else if (st.state == FW_DOWNLOAD_CHECKED || st.state == FW_DOWNLOAD_ERROR) {
                return FW_MENU_RETURN_FORM;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}
