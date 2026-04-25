#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "config.h"
#include "litewing_espnow.h"
#include "pm_esplane.h"
#include "stabilizer.h"
#include "system.h"
#include "wifi_esp32.h"

#define DEBUG_MODULE "LW_ESPNOW"
#include "debug_cf.h"

#ifndef CONFIG_LITEWING_ESPNOW_TELEMETRY_PERIOD_MS
#define CONFIG_LITEWING_ESPNOW_TELEMETRY_PERIOD_MS 200
#endif

static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static bool isInit;
static uint32_t txSequence;

static void litewingEspnowRecvCb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;

#ifdef CONFIG_LITEWING_ENABLE_ESPNOW_COMMAND_RX
    if (len == sizeof(litewing_command_packet_t)) {
        litewing_command_packet_t command;
        memcpy(&command, data, sizeof(command));

        if (memcmp(command.magic, LITEWING_COMMAND_MAGIC, sizeof(command.magic)) == 0 &&
                command.version == LITEWING_ESPNOW_VERSION) {
            wifiInjectRxPacket(sizeof(command), (const uint8_t *)&command);
        }
    } else if (len == 7 && data[0] == 'n' && data[1] == 'o' && data[2] == 'w') {
        wifiInjectRxPacket((uint32_t)len, data);
    }
#else
    (void)data;
    (void)len;
#endif
}

static void litewingEspnowSendCb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    (void)mac_addr;
    (void)status;
}

static esp_err_t litewingEspnowAddBroadcastPeer(void)
{
    if (esp_now_is_peer_exist(broadcast_mac)) {
        return ESP_OK;
    }

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, broadcast_mac, sizeof(peer.peer_addr));
    peer.channel = CONFIG_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_AP;
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_ERR_ESPNOW_EXIST) {
        return ESP_OK;
    }

    return err;
}

static void litewingTelemetryTask(void *param)
{
    (void)param;

    systemWaitStart();

    while (true) {
#ifdef CONFIG_LITEWING_ENABLE_ESPNOW_TELEMETRY
        state_t state;
        control_t control;
        stabilizerGetLatestState(&state, NULL, &control);

        char payload[ESP_NOW_MAX_DATA_LEN] = {0};
        int len = snprintf(payload, sizeof(payload),
                           "{\"type\":\"telemetry\",\"seq\":%" PRIu32 ",\"ms\":%" PRIu32
                           ",\"armed\":%d,\"battery\":%.2f,\"roll\":%.2f,\"pitch\":%.2f"
                           ",\"yaw\":%.2f,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f"
                           ",\"vx\":%.2f,\"vy\":%.2f,\"vz\":%.2f,\"thrust\":%.0f}",
                           txSequence++,
                           (uint32_t)(esp_timer_get_time() / 1000),
                           systemIsArmed() ? 1 : 0,
                           (double)pmGetBatteryVoltage(),
                           (double)state.attitude.roll,
                           (double)state.attitude.pitch,
                           (double)state.attitude.yaw,
                           (double)state.position.x,
                           (double)state.position.y,
                           (double)state.position.z,
                           (double)state.velocity.x,
                           (double)state.velocity.y,
                           (double)state.velocity.z,
                           (double)control.thrust);

        if (len > 0 && len < (int)sizeof(payload)) {
            esp_err_t err = esp_now_send(broadcast_mac, (const uint8_t *)payload, len);
            if (err != ESP_OK) {
                DEBUG_PRINT_LOCAL("telemetry send failed: %s", esp_err_to_name(err));
            }
        } else {
            DEBUG_PRINT_LOCAL("telemetry payload truncated");
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(CONFIG_LITEWING_ESPNOW_TELEMETRY_PERIOD_MS));
    }
}

esp_err_t litewingEspnowInit(void)
{
    if (isInit) {
        return ESP_OK;
    }

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        return err;
    }

    ESP_ERROR_CHECK(esp_now_register_recv_cb(litewingEspnowRecvCb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(litewingEspnowSendCb));
    ESP_ERROR_CHECK(litewingEspnowAddBroadcastPeer());

    xTaskCreate(litewingTelemetryTask, "LW_ESPNOW_TX", 4096, NULL, UDP_TX_TASK_PRI, NULL);
    isInit = true;

    DEBUG_PRINT_LOCAL("ESP-NOW telemetry/command initialized on channel %d", CONFIG_WIFI_CHANNEL);
    return ESP_OK;
}
