/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <zephyr/device.h>

/**
 * @brief Test whether @p dev is an instance of the
 * zmk,input-processor-touch-stream-scroll marker processor.
 *
 * Defined in input_processor_touch_stream_scroll.c; compiled only when at
 * least one marker node is referenced in the devicetree.
 */
#if IS_ENABLED(CONFIG_ZMK_INPUT_PROCESSOR_TOUCH_STREAM_SCROLL)
bool zmk_input_processor_is_touch_stream_scroll(const struct device *dev);
#else
static inline bool zmk_input_processor_is_touch_stream_scroll(const struct device *dev) {
    ARG_UNUSED(dev);
    return false;
}
#endif

#if IS_ENABLED(CONFIG_ZMK_TOUCH_STREAM)

/**
 * @brief Evaluate whether a scroll-marker processor is currently in the
 * effective processor chain for events from input device @p input_dev.
 *
 * Mirrors the input listener's own routing rules: layer overlays are
 * checked in devicetree order; an overlay counts if any of its layers is
 * active, and an active overlay without process-next shadows everything
 * after it (including the base chain), exactly as it does for real events.
 * The check is driven purely by current layer state, so it is valid before
 * any event has been processed (e.g. on the first frame of a touch while
 * the layer was already held).
 *
 * Defined in input_listener.c.
 */
bool zmk_input_listener_touch_stream_scroll_active(const struct device *input_dev);

#endif // IS_ENABLED(CONFIG_ZMK_TOUCH_STREAM)
