/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Raw touch streaming (protocol v2): forwards absolute touch frames
 * (position + touch strength) from a Cirque Pinnacle trackpad in absolute
 * mode to the host over a vendor-defined HID report (see
 * ZMK_HID_REPORT_ID_TOUCH_STREAM).
 *
 * One report is emitted per Pinnacle sample while touched (~100 Hz), plus
 * exactly one release report (touched = 0, z = 0) on lift-off. An 8-byte
 * feature report on the same report ID describes the pad (protocol
 * version, pads present, resolution, orientation, coordinate ranges) and
 * is readable over USB GET_REPORT and the BLE HOG feature-report
 * characteristic.
 *
 * Scroll mode: frames carry the scroll-mode flag while a processor chain
 * containing the zmk,input-processor-touch-stream-scroll marker would
 * handle the pad's events (an active listener layer overlay, or the base
 * chain - see zmk_input_listener_touch_stream_scroll_active()). The check
 * is driven by layer state alone, so the flag is correct from the first
 * frame of a touch, including when the layer was held before touch-down,
 * and follows layer changes mid-touch.
 *
 * Dual mode: relative deltas derived from successive absolute positions
 * are ALWAYS re-injected as REL_X/REL_Y on the same input device,
 * regardless of scroll mode, so the existing input listener chain
 * (scaling, wheel-mapping overlays, temp mouse layer, buttons) keeps
 * working as a fallback. A host consuming the raw stream is expected to
 * suppress the pointer/wheel events it supersedes.
 *
 * Tap-to-click: when the pad node sets stream-tap-click, a touch that
 * lifts off within stream-tap-max-ms and never strays more than
 * stream-tap-max-movement raw counts (Chebyshev distance) from its
 * touch-down point - and never had a scroll-mode frame - injects an
 * INPUT_BTN_0 press + release into the pad's normal input pipeline, so
 * existing button processors apply.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/endpoints.h>
#include <zmk/hid.h>
#include <zmk/keymap.h>
#include <zmk/pointing/touch_stream.h>

#define TOUCH_STREAM_PAD_ID 0

#define TOUCH_DEV_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(cirque_pinnacle)

static const struct device *const touch_dev = DEVICE_DT_GET(TOUCH_DEV_NODE);

/* The Pinnacle's invert/rotate feed transforms only apply to its relative
 * mode, so mirror them here when deriving pointer deltas from the raw
 * absolute coordinates. The streamed frames stay untransformed; the same
 * property values are exposed verbatim in the feature report's orientation
 * byte so hosts can apply them. */
#define TOUCH_ROTATE_90 DT_PROP(TOUCH_DEV_NODE, rotate_90)
#define TOUCH_X_INVERT DT_PROP(TOUCH_DEV_NODE, x_invert)
#define TOUCH_Y_INVERT DT_PROP(TOUCH_DEV_NODE, y_invert)

/* Firmware tap-to-click configuration (pad devicetree, default off). */
#define TOUCH_TAP_CLICK DT_PROP(TOUCH_DEV_NODE, stream_tap_click)
#define TOUCH_TAP_MAX_MS DT_PROP(TOUCH_DEV_NODE, stream_tap_max_ms)
#define TOUCH_TAP_MAX_MOVEMENT DT_PROP(TOUCH_DEV_NODE, stream_tap_max_movement)

/* Fixed Pinnacle ASIC characteristics for the feature report. The driver
 * exposes no devicetree properties for these; values are from Cirque's
 * GlidePoint absolute-mode documentation (~38 counts/mm). */
#define TOUCH_STREAM_ABS_X_MAX 2047
#define TOUCH_STREAM_ABS_Y_MAX 1535
#define TOUCH_STREAM_RESOLUTION_CPMM 38

/* Current frame, accumulated until the sync event. */
static uint16_t cur_x, cur_y;
static uint8_t cur_z;

/* Previous frame state for release dedup and delta derivation. */
static bool prev_touched;
static bool have_prev_pos;
static uint16_t prev_x, prev_y;

/* Tap detection state for the current touch. */
static int64_t touch_down_ts;
static uint16_t touch_down_x, touch_down_y;
static bool tap_candidate;
static bool tap_scroll_seen;

static bool touch_stream_scroll_context(void) {
#if IS_ENABLED(CONFIG_ZMK_INPUT_LISTENER)
    return zmk_input_listener_touch_stream_scroll_active(touch_dev);
#else
    return false;
#endif
}

static void touch_stream_emit_tap(void) {
    /* K_NO_WAIT: dropping a click beats deadlocking the input queue we
     * are dispatched from. Press and release are separate sync'd events,
     * so the listener sends a button-down report followed by a button-up
     * report through the pad's normal processor chain. */
    input_report_key(touch_dev, INPUT_BTN_0, 1, true, K_NO_WAIT);
    input_report_key(touch_dev, INPUT_BTN_0, 0, true, K_NO_WAIT);
}

static void touch_stream_process_frame(void) {
    /* The Pinnacle marks lift-off with an all-zeros Z-idle frame. */
    bool touched = !(cur_x == 0 && cur_y == 0 && cur_z == 0);

    if (!touched && !prev_touched) {
        /* Nothing to stream while idle; also swallows any extra Z-idle
         * frames so exactly one release report goes out. */
        return;
    }

    bool scroll_mode = touch_stream_scroll_context();

    uint8_t flags = (touched ? ZMK_HID_TOUCH_STREAM_FLAGS_TOUCHED : 0) |
                    (scroll_mode ? ZMK_HID_TOUCH_STREAM_FLAGS_SCROLL_MODE : 0);

    zmk_hid_touch_stream_set(TOUCH_STREAM_PAD_ID, touched ? cur_x : 0, touched ? cur_y : 0,
                             touched ? cur_z : 0, flags);
    zmk_endpoints_send_touch_stream_report();

    if (touched && !prev_touched) {
        /* Touch-down: start a tap candidacy. */
        touch_down_ts = k_uptime_get();
        touch_down_x = cur_x;
        touch_down_y = cur_y;
        tap_candidate = TOUCH_TAP_CLICK;
        tap_scroll_seen = false;
    }

    tap_scroll_seen = tap_scroll_seen || scroll_mode;

    if (touched && tap_candidate) {
        int32_t travel_x = (int32_t)cur_x - (int32_t)touch_down_x;
        int32_t travel_y = (int32_t)cur_y - (int32_t)touch_down_y;
        if (MAX(travel_x < 0 ? -travel_x : travel_x, travel_y < 0 ? -travel_y : travel_y) >
            TOUCH_TAP_MAX_MOVEMENT) {
            tap_candidate = false;
        }
    }

    /* Dual mode: always derive relative deltas for the normal pointer
     * pipeline, even in scroll mode - an existing wheel-mapping overlay
     * then provides standard wheel scrolling as a fallback. Hosts that
     * consume the raw stream suppress those events themselves. */
    if (touched && have_prev_pos) {
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

    if (!touched && tap_candidate && !tap_scroll_seen &&
        (k_uptime_get() - touch_down_ts) <= TOUCH_TAP_MAX_MS) {
        touch_stream_emit_tap();
    }

    if (touched) {
        prev_x = cur_x;
        prev_y = cur_y;
        have_prev_pos = true;
    } else {
        have_prev_pos = false;
        tap_candidate = false;
    }
    prev_touched = touched;
}

static void touch_stream_input_cb(struct input_event *evt) {
    if (evt->type != INPUT_EV_ABS) {
        /* Ignore everything else, including our own injected REL/KEY events. */
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

static int touch_stream_init(void) {
    uint8_t orientation = (TOUCH_ROTATE_90 ? ZMK_HID_TOUCH_STREAM_ORIENT_ROTATE_90 : 0) |
                          (TOUCH_X_INVERT ? ZMK_HID_TOUCH_STREAM_ORIENT_X_INVERT : 0) |
                          (TOUCH_Y_INVERT ? ZMK_HID_TOUCH_STREAM_ORIENT_Y_INVERT : 0);

    zmk_hid_touch_stream_set_feature(BIT(TOUCH_STREAM_PAD_ID), TOUCH_STREAM_RESOLUTION_CPMM,
                                     orientation, TOUCH_STREAM_ABS_X_MAX, TOUCH_STREAM_ABS_Y_MAX);

    return 0;
}

SYS_INIT(touch_stream_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
