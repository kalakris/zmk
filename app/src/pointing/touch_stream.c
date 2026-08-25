/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Raw touch streaming: forwards absolute touch frames (position + touch
 * strength) from a Cirque Pinnacle trackpad in absolute mode to the host
 * over a vendor-defined HID report (see ZMK_HID_REPORT_ID_TOUCH_STREAM).
 *
 * One report is emitted per Pinnacle sample while touched (~100 Hz), plus
 * exactly one release report (touched = 0, z = 0) on lift-off. Frames carry
 * a "scroll mode" flag while the configured scroll layer
 * (CONFIG_ZMK_TOUCH_STREAM_SCROLL_LAYER) is active; in that state the pad's
 * motion is withheld from the normal pointer pipeline so the host can
 * synthesize scrolling from the raw frames. Outside scroll mode, relative
 * deltas are derived from successive absolute positions and re-injected as
 * REL_X/REL_Y on the same input device, so the existing input listener
 * chain (scaling, temp mouse layer, buttons) keeps working unchanged.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/endpoints.h>
#include <zmk/hid.h>
#include <zmk/keymap.h>

#define TOUCH_STREAM_PAD_ID 0

#define TOUCH_DEV_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(cirque_pinnacle)

static const struct device *const touch_dev = DEVICE_DT_GET(TOUCH_DEV_NODE);

/* The Pinnacle's invert/rotate feed transforms only apply to its relative
 * mode, so mirror them here when deriving pointer deltas from the raw
 * absolute coordinates. The streamed frames stay untransformed. */
#define TOUCH_ROTATE_90 DT_PROP(TOUCH_DEV_NODE, rotate_90)
#define TOUCH_X_INVERT DT_PROP(TOUCH_DEV_NODE, x_invert)
#define TOUCH_Y_INVERT DT_PROP(TOUCH_DEV_NODE, y_invert)

/* Current frame, accumulated until the sync event. */
static uint16_t cur_x, cur_y;
static uint8_t cur_z;

/* Previous frame state for release dedup and delta derivation. */
static bool prev_touched;
static bool have_prev_pos;
static uint16_t prev_x, prev_y;

static void touch_stream_process_frame(void) {
    /* The Pinnacle marks lift-off with an all-zeros Z-idle frame. */
    bool touched = !(cur_x == 0 && cur_y == 0 && cur_z == 0);

    if (!touched && !prev_touched) {
        /* Nothing to stream while idle; also swallows any extra Z-idle
         * frames so exactly one release report goes out. */
        return;
    }

    /* NOTE: the scroll layer is checked with zmk_keymap_layer_active()
     * rather than zmk_keymap_highest_layer_active(), because a trackpad
     * driven temp mouse layer (zip_temp_layer) may sit above the scroll
     * layer while the pad is in use. */
    bool scroll_mode = zmk_keymap_layer_active(CONFIG_ZMK_TOUCH_STREAM_SCROLL_LAYER);

    uint8_t flags = (touched ? ZMK_HID_TOUCH_STREAM_FLAGS_TOUCHED : 0) |
                    (scroll_mode ? ZMK_HID_TOUCH_STREAM_FLAGS_SCROLL_MODE : 0);

    zmk_hid_touch_stream_set(TOUCH_STREAM_PAD_ID, touched ? cur_x : 0, touched ? cur_y : 0,
                             touched ? cur_z : 0, flags);
    zmk_endpoints_send_touch_stream_report();

    if (touched && !scroll_mode && have_prev_pos) {
        int32_t raw_dx = (int32_t)cur_x - (int32_t)prev_x;
        int32_t raw_dy = (int32_t)cur_y - (int32_t)prev_y;

        int32_t dx = TOUCH_ROTATE_90 ? raw_dy : raw_dx;
        int32_t dy = TOUCH_ROTATE_90 ? raw_dx : raw_dy;
        if (TOUCH_X_INVERT) {
            dx = -dx;
        }
        if (TOUCH_Y_INVERT) {
            dy = -dy;
        }

        if (dx != 0 || dy != 0) {
            /* K_NO_WAIT: dropping a delta beats deadlocking the input
             * queue we are dispatched from. */
            input_report_rel(touch_dev, INPUT_REL_X, dx, false, K_NO_WAIT);
            input_report_rel(touch_dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
        }
    }

    if (touched) {
        prev_x = cur_x;
        prev_y = cur_y;
        have_prev_pos = true;
    } else {
        have_prev_pos = false;
    }
    prev_touched = touched;
}

static void touch_stream_input_cb(struct input_event *evt) {
    if (evt->type != INPUT_EV_ABS) {
        /* Ignore everything else, including our own injected REL events. */
        return;
    }

    switch (evt->code) {
    case INPUT_ABS_X:
        cur_x = (uint16_t)evt->value;
        break;
    case INPUT_ABS_Y:
        cur_y = (uint16_t)evt->value;
        break;
    case INPUT_ABS_Z:
        cur_z = (uint8_t)evt->value;
        break;
    default:
        return;
    }

    if (evt->sync) {
        touch_stream_process_frame();
    }
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(TOUCH_DEV_NODE), touch_stream_input_cb);
