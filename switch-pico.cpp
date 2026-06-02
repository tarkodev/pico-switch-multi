#include <stdio.h>
#include <string.h>
#include "bsp/board.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "switch_pro_driver.h"

#ifdef SWITCH_PICO_LOG
#define LOG_PRINTF(...) printf(__VA_ARGS__)
#else
#define LOG_PRINTF(...) ((void)0)
#endif

// UART1 is reserved for external input frames from the host PC.
#define UART_ID uart1
#define BAUD_RATE 921600
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define UART_RUMBLE_HEADER 0xBB
#define UART_RUMBLE_RUMBLE_TYPE 0x01
#define UART_PROTOCOL_SINGLE 0x02
#define UART_PROTOCOL_MULTI 0x03
#define UART_PROTOCOL_CONTROL 0x04
#define UART_CONTROL_ANNOUNCE 0x01
#define UART_CONTROL_SET_COLOR 0x02
#define BRIDGE_ACTIVE_TIMEOUT_MS 1000u
#define USB_COLOR_RECONNECT_DELAY_MS 250u
#define ANNOUNCE_DELAY_MS 1000u
#define ANNOUNCE_PULSE_MS 100u
#ifndef SWITCH_PICO_CONTROLLER_COUNT
#define SWITCH_PICO_CONTROLLER_COUNT 8
#endif

static bool g_last_mounted = false;
static bool g_last_ready[SWITCH_PICO_CONTROLLER_COUNT] = {};
static bool g_usb_stack_started = false;
static bool g_usb_connected_by_bridge = false;
static uint32_t g_bridge_active_until_ms = 0;
static uint32_t g_usb_reconnect_at_ms = 0;
static uint16_t g_pending_announce_ms[SWITCH_PICO_CONTROLLER_COUNT] = {};

// Track the latest state provided by UART or the autopilot.
static SwitchInputState g_user_state[SWITCH_PICO_CONTROLLER_COUNT];
static uint32_t g_announce_started_at_ms[SWITCH_PICO_CONTROLLER_COUNT] = {};
static uint32_t g_announce_until_ms[SWITCH_PICO_CONTROLLER_COUNT] = {};

static void init_uart_input() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
}

static SwitchInputState neutral_input() {
    SwitchInputState state{};
    state.lx = SWITCH_PRO_JOYSTICK_MID;
    state.ly = SWITCH_PRO_JOYSTICK_MID;
    state.rx = SWITCH_PRO_JOYSTICK_MID;
    state.ry = SWITCH_PRO_JOYSTICK_MID;
    return state;
}

static void send_rumble_uart_frame(const uint8_t rumble[8]) {
    uint8_t frame[11];
    frame[0] = UART_RUMBLE_HEADER;
    frame[1] = UART_RUMBLE_RUMBLE_TYPE;
    memcpy(&frame[2], rumble, 8);

    uint8_t checksum = 0;
    for (int i = 0; i < 10; ++i) {
        checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    frame[10] = checksum;
    uart_write_blocking(UART_ID, frame, sizeof(frame));
}

static void on_rumble_from_switch(const uint8_t rumble[8]) {
    send_rumble_uart_frame(rumble);
}

// Consume UART bytes and forward complete frames to the Switch Pro driver.
static uint8_t uart_checksum8(const uint8_t* data, uint16_t length) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; ++i) {
        checksum = static_cast<uint8_t>(checksum + data[i]);
    }
    return checksum;
}

static uint32_t now_ms() {
    return to_ms_since_boot(get_absolute_time());
}

static void mark_bridge_active() {
    g_bridge_active_until_ms = now_ms() + BRIDGE_ACTIVE_TIMEOUT_MS;
}

static bool bridge_is_active(uint32_t current_ms) {
    return g_bridge_active_until_ms != 0 && current_ms < g_bridge_active_until_ms;
}

static void force_usb_color_reconnect() {
    if (g_usb_stack_started && (g_usb_connected_by_bridge || tud_mounted() || tud_connected())) {
        tud_disconnect();
        g_usb_connected_by_bridge = false;
        g_usb_reconnect_at_ms = now_ms() + USB_COLOR_RECONNECT_DELAY_MS;
        LOG_PRINTF("[USB] color reconnect scheduled\n");
    }
}

static void update_usb_bridge_connection() {
    uint32_t current_ms = now_ms();
    if (bridge_is_active(current_ms)) {
        if (!g_usb_connected_by_bridge && (g_usb_reconnect_at_ms == 0 || current_ms >= g_usb_reconnect_at_ms)) {
            if (!g_usb_stack_started) {
                tusb_init();
                g_usb_stack_started = true;
                LOG_PRINTF("[USB] stack started\n");
            }
            tud_connect();
            g_usb_connected_by_bridge = true;
            g_usb_reconnect_at_ms = 0;
            LOG_PRINTF("[USB] bridge active -> connect\n");
        }
        return;
    }

    if (g_usb_connected_by_bridge) {
        tud_disconnect();
        g_usb_connected_by_bridge = false;
        g_usb_reconnect_at_ms = 0;
        for (uint8_t i = 0; i < SWITCH_PICO_CONTROLLER_COUNT; ++i) {
            g_user_state[i] = neutral_input();
            g_announce_started_at_ms[i] = 0;
            g_announce_until_ms[i] = 0;
            g_pending_announce_ms[i] = 0;
            switch_pro_set_input_for(i, g_user_state[i]);
        }
        LOG_PRINTF("[USB] bridge inactive -> disconnect\n");
    }
}

static bool parse_uart_controller_packet(const uint8_t* packet, uint8_t length, uint8_t* controller_id, SwitchInputState* parsed) {
    if (!packet || !controller_id || !parsed || length < 12 || packet[0] != 0xAA) {
        return false;
    }

    // Backwards compatible v2: no controller id, always slot 0.
    if (packet[1] == UART_PROTOCOL_SINGLE) {
        *controller_id = 0;
        return switch_pro_apply_uart_packet(packet, length, parsed);
    }

    // v3 input: 0xAA, 0x03, controller_id, payload_len, payload..., checksum.
    if (packet[1] != UART_PROTOCOL_MULTI || length < 13) {
        return false;
    }

    uint8_t id = packet[2];
    if (id >= SWITCH_PICO_CONTROLLER_COUNT) {
        return false;
    }

    uint8_t payload_len = packet[3];
    if (static_cast<uint16_t>(payload_len) + 5u != length) {
        return false;
    }

    if (uart_checksum8(packet, static_cast<uint16_t>(4u + payload_len)) != packet[length - 1]) {
        return false;
    }

    // Rebuild a v2 packet so the existing parser remains the single source of truth.
    uint8_t v2[64];
    if (static_cast<uint16_t>(payload_len) + 4u > sizeof(v2)) {
        return false;
    }
    v2[0] = 0xAA;
    v2[1] = UART_PROTOCOL_SINGLE;
    v2[2] = payload_len;
    memcpy(&v2[3], &packet[4], payload_len);
    v2[3 + payload_len] = uart_checksum8(v2, static_cast<uint16_t>(3u + payload_len));

    if (!switch_pro_apply_uart_packet(v2, static_cast<uint8_t>(payload_len + 4u), parsed)) {
        return false;
    }
    *controller_id = id;
    return true;
}

static bool handle_uart_control_packet(const uint8_t* packet, uint8_t length) {
    // Control: 0xAA, 0x04, command, slot_id, payload_len, payload..., checksum.
    if (!packet || length < 6 || packet[0] != 0xAA || packet[1] != UART_PROTOCOL_CONTROL) {
        return false;
    }
    uint8_t command = packet[2];
    uint8_t slot_id = packet[3];
    uint8_t payload_len = packet[4];
    if (slot_id >= SWITCH_PICO_CONTROLLER_COUNT) {
        return false;
    }
    if (static_cast<uint16_t>(payload_len) + 6u != length) {
        return false;
    }
    if (uart_checksum8(packet, static_cast<uint16_t>(5u + payload_len)) != packet[length - 1]) {
        return false;
    }

    const uint8_t* payload = &packet[5];
    switch (command) {
        case UART_CONTROL_ANNOUNCE: {
            uint16_t duration_ms = 120;
            if (payload_len >= 2) {
                duration_ms = static_cast<uint16_t>(payload[0]) | (static_cast<uint16_t>(payload[1]) << 8);
            }
            if (duration_ms < 20) duration_ms = 20;
            if (duration_ms > 1000) duration_ms = 1000;
            if (tud_mounted()) {
                uint32_t start_ms = now_ms();
                g_announce_started_at_ms[slot_id] = start_ms + ANNOUNCE_DELAY_MS;
                g_announce_until_ms[slot_id] = start_ms + ANNOUNCE_DELAY_MS + ANNOUNCE_PULSE_MS;
            } else {
                g_pending_announce_ms[slot_id] = duration_ms;
            }
            LOG_PRINTF("[UART] announce slot=%u duration=%u\n", slot_id, duration_ms);
            return true;
        }
        case UART_CONTROL_SET_COLOR: {
            if (payload_len < 6) {
                return false;
            }
            switch_pro_set_color_for(slot_id, payload[0], payload[1], payload[2], payload[3], payload[4], payload[5]);
            force_usb_color_reconnect();
            LOG_PRINTF("[UART] color slot=%u body=%02x%02x%02x buttons=%02x%02x%02x\n",
                slot_id, payload[0], payload[1], payload[2], payload[3], payload[4], payload[5]);
            return true;
        }
        default:
            LOG_PRINTF("[UART] unknown control command=%u slot=%u\n", command, slot_id);
            return false;
    }
}

static bool poll_uart_frames() {
    static uint8_t buffer[96];
    static uint8_t index = 0;
    static uint8_t expected_len = 0;
    static absolute_time_t last_byte_time = {0};
    static bool has_last_byte = false;
    bool new_data = false;

    while (uart_is_readable(UART_ID)) {
        uint8_t byte = uart_getc(UART_ID);
        uint64_t now = now_ms();
        if (has_last_byte && (now - to_ms_since_boot(last_byte_time)) > 20) {
            index = 0;
            expected_len = 0;
        }
        last_byte_time = get_absolute_time();
        has_last_byte = true;

        if (index == 0 && byte != 0xAA) {
            continue;
        }
        if (index >= sizeof(buffer)) {
            index = 0;
            expected_len = 0;
        }

        buffer[index++] = byte;

        if (index == 2 && buffer[1] != UART_PROTOCOL_SINGLE && buffer[1] != UART_PROTOCOL_MULTI && buffer[1] != UART_PROTOCOL_CONTROL) {
            index = 0;
            expected_len = 0;
            continue;
        }

        if (buffer[1] == UART_PROTOCOL_SINGLE && index == 3) {
            expected_len = static_cast<uint8_t>(buffer[2] + 4u);
            if (expected_len < 12 || expected_len > sizeof(buffer)) {
                index = 0;
                expected_len = 0;
                continue;
            }
        } else if (buffer[1] == UART_PROTOCOL_MULTI && index == 4) {
            expected_len = static_cast<uint8_t>(buffer[3] + 5u);
            if (expected_len < 13 || expected_len > sizeof(buffer)) {
                index = 0;
                expected_len = 0;
                continue;
            }
        } else if (buffer[1] == UART_PROTOCOL_CONTROL && index == 5) {
            expected_len = static_cast<uint8_t>(buffer[4] + 6u);
            if (expected_len < 6 || expected_len > sizeof(buffer)) {
                index = 0;
                expected_len = 0;
                continue;
            }
        }

        if (expected_len > 0 && index >= expected_len) {
            if (buffer[1] == UART_PROTOCOL_CONTROL) {
                if (handle_uart_control_packet(buffer, expected_len)) {
                    mark_bridge_active();
                }
            } else {
                uint8_t controller_id = 0;
                SwitchInputState parsed{};
                if (parse_uart_controller_packet(buffer, expected_len, &controller_id, &parsed)) {
                    g_user_state[controller_id] = parsed;
                    mark_bridge_active();
                    new_data = true;
                    LOG_PRINTF("[UART] slot=%u input packet\n", controller_id);
                }
            }
            index = 0;
            expected_len = 0;
        }
    }
    return new_data;
}

static void log_usb_state() {
    if (!g_usb_stack_started) {
        return;
    }
    bool mounted = tud_mounted();
    if (mounted != g_last_mounted) {
        g_last_mounted = mounted;
        LOG_PRINTF("[USB] %s\n", mounted ? "mounted" : "unmounted");
    }

    for (uint8_t i = 0; i < SWITCH_PICO_CONTROLLER_COUNT; ++i) {
        bool ready = switch_pro_is_ready_for(i);
        if (ready != g_last_ready[i]) {
            g_last_ready[i] = ready;
            LOG_PRINTF("[SWITCH] slot %u driver %s\n", i, ready ? "ready" : "not ready");
        }
    }
}

int main() {
    board_init();
    stdio_init_all();

    init_uart_input();

    switch_pro_init();
    switch_pro_set_rumble_callback(on_rumble_from_switch);
    for (uint8_t i = 0; i < SWITCH_PICO_CONTROLLER_COUNT; ++i) {
        g_user_state[i] = neutral_input();
        switch_pro_set_input_for(i, g_user_state[i]);
    }

    LOG_PRINTF("[BOOT] switch-pico starting (UART0 log @ 115200)\n");
    LOG_PRINTF("[INFO] UART1 pins TX=%d RX=%d baud=%d\n",
           UART_TX_PIN, UART_RX_PIN, BAUD_RATE);

    while (true) {
        bool new_data = poll_uart_frames();  // Pull controller state from UART1
        (void)new_data;
        update_usb_bridge_connection();
        if (g_usb_stack_started) {
            tud_task();      // USB device tasks
        }
        uint32_t loop_now_ms = now_ms();
        for (uint8_t i = 0; i < SWITCH_PICO_CONTROLLER_COUNT; ++i) {
            SwitchInputState state = g_user_state[i];
            if (g_pending_announce_ms[i] != 0 && tud_mounted()) {
                g_announce_started_at_ms[i] = loop_now_ms + ANNOUNCE_DELAY_MS;
                g_announce_until_ms[i] = loop_now_ms + ANNOUNCE_DELAY_MS + ANNOUNCE_PULSE_MS;
                g_pending_announce_ms[i] = 0;
                LOG_PRINTF("[UART] announce armed slot=%u\n", i);
            }
            if (g_announce_until_ms[i] != 0 && loop_now_ms < g_announce_until_ms[i]) {
                if (loop_now_ms >= g_announce_started_at_ms[i]) {
                    state.button_l = true;
                    state.button_r = true;
                }
            } else if (g_announce_until_ms[i] != 0 && loop_now_ms >= g_announce_until_ms[i]) {
                g_announce_started_at_ms[i] = 0;
                g_announce_until_ms[i] = 0;
            }
            switch_pro_set_input_for(i, state);
        }
        if (g_usb_stack_started) {
            switch_pro_task();   // Push state to the Switch host
        }
        log_usb_state();
    }
}
