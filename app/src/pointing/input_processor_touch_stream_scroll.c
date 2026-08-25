/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Marker input processor for the raw touch stream (see touch_stream.c).
 *
 * The processor itself is a pure pass-through: it never modifies or stops
 * events. Its only purpose is to mark a "scroll context" in the devicetree:
 * when an input listener chain containing this processor would currently
 * handle events from a streaming pad (base chain, or a layer overlay whose
 * layer is active), the touch-stream module sets the scroll-mode flag on
 * that pad's streamed frames. The evaluation happens in input_listener.c
 * (zmk_input_listener_touch_stream_scroll_active()), which needs a way to
 * recognize instances of this processor - hence the instance registry here.
 */

#define DT_DRV_COMPAT zmk_input_processor_touch_stream_scroll

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>

#include <zmk/pointing/touch_stream.h>

static int tss_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                            uint32_t param2, struct zmk_input_processor_state *state) {
    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api tss_driver_api = {
    .handle_event = tss_handle_event,
};

static int tss_init(const struct device *dev) { return 0; }

#define TSS_INST(n)                                                                                \
    DEVICE_DT_INST_DEFINE(n, &tss_init, NULL, NULL, NULL, POST_KERNEL,                             \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &tss_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TSS_INST)

#define TSS_DEV_REF(n) DEVICE_DT_INST_GET(n),

static const struct device *const tss_devices[] = {DT_INST_FOREACH_STATUS_OKAY(TSS_DEV_REF)};

bool zmk_input_processor_is_touch_stream_scroll(const struct device *dev) {
    for (size_t i = 0; i < ARRAY_SIZE(tss_devices); i++) {
        if (tss_devices[i] == dev) {
            return true;
        }
    }

    return false;
}
