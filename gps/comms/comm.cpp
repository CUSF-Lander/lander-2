#include "comm.h"         // WIFI namespace — also pulls in IDF's esp_now.h
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include <string.h>
#include <stddef.h>       // offsetof

static const char* TAG = "WIFI";

namespace WIFI {

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

static CorrectionCallback s_correction_cb = nullptr;
static GpsCallback        s_gps_cb        = nullptr;

static uint8_t s_peer_mac[6] = {};
static bool    s_peer_known  = false;

// ---------------------------------------------------------------------------
// Internal ESP-NOW callbacks
// ---------------------------------------------------------------------------

static void recv_cb(const esp_now_recv_info_t* recv_info,
                    const uint8_t* data, int len)
{
    if (!recv_info || !data || len < 1) return;

    const uint8_t* src = recv_info->src_addr;

    // Auto-learn: on the first received frame, register the sender as the
    // peer we reply to.  This lets both sides bootstrap without hard-coding MACs.
    if (!s_peer_known) {
        memcpy(s_peer_mac, src, 6);
        s_peer_known = true;

        // add_peer handles the case where it is already registered
        add_peer(s_peer_mac);

        ESP_LOGI(TAG, "Auto-learned peer: %02x:%02x:%02x:%02x:%02x:%02x",
                 s_peer_mac[0], s_peer_mac[1], s_peer_mac[2],
                 s_peer_mac[3], s_peer_mac[4], s_peer_mac[5]);
    }

    auto type = static_cast<PacketType>(data[0]);

    switch (type) {

        case PacketType::CORRECTION: {
            if (len < 2) {
                ESP_LOGW(TAG, "CORRECTION packet too short (%d bytes)", len);
                break;
            }
            const auto* pkt = reinterpret_cast<const CorrectionPacket*>(data);
            if (s_correction_cb) {
                s_correction_cb(pkt->data, pkt->len, src);
            }
            break;
        }

        case PacketType::GPS_POSITION: {
            if (len < static_cast<int>(sizeof(GpsPacket))) {
                ESP_LOGW(TAG, "GPS_POSITION packet too short (%d bytes)", len);
                break;
            }
            const auto* pkt = reinterpret_cast<const GpsPacket*>(data);
            if (s_gps_cb) {
                s_gps_cb(pkt->position, src);
            }
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown packet type: 0x%02x (len=%d)", data[0], len);
            break;
    }
}

static void send_cb(const uint8_t* mac_addr, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "Send failed to %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    }
}

// ---------------------------------------------------------------------------
// Public API — Initialisation
// ---------------------------------------------------------------------------

esp_err_t wifi_init()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Enable Long-Range (LR) mode alongside the standard protocols so both
    // ESPs can communicate over longer distances.
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "WiFi initialised (STA + LR, channel 1)");
    return ESP_OK;
}

esp_err_t esp_now_init()
{
    // Call IDF's esp_now_init via the global scope to avoid ambiguity with
    // our own WIFI::esp_now_init().
    esp_err_t err = ::esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(::esp_now_register_recv_cb(recv_cb));
    ESP_ERROR_CHECK(::esp_now_register_send_cb(send_cb));

    ESP_LOGI(TAG, "ESP-NOW initialised");
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Public API — Peer management
// ---------------------------------------------------------------------------

esp_err_t add_peer(const uint8_t* mac_addr)
{
    // Register with ESP-NOW only if not already present
    if (!::esp_now_is_peer_exist(mac_addr)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mac_addr, 6);
        peer.channel = 1;
        peer.ifidx   = WIFI_IF_STA;
        peer.encrypt = false;

        esp_err_t err = ::esp_now_add_peer(&peer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "Peer registered: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    }

    // Set (or update) the send-target MAC
    memcpy(s_peer_mac, mac_addr, 6);
    s_peer_known = true;

    return ESP_OK;
}

bool is_peer_known()
{
    return s_peer_known;
}

const uint8_t* get_peer_mac()
{
    return s_peer_known ? s_peer_mac : nullptr;
}

// ---------------------------------------------------------------------------
// Public API — Sending
// ---------------------------------------------------------------------------

esp_err_t send_correction(const uint8_t* data, uint8_t len)
{
    if (!s_peer_known) {
        ESP_LOGE(TAG, "send_correction: no peer known yet");
        return ESP_ERR_INVALID_STATE;
    }
    if (len > MAX_CORRECTION_LEN) {
        ESP_LOGE(TAG, "send_correction: len %u exceeds MAX_CORRECTION_LEN %u",
                 len, MAX_CORRECTION_LEN);
        return ESP_ERR_INVALID_ARG;
    }

    CorrectionPacket pkt;
    pkt.type = PacketType::CORRECTION;
    pkt.len  = len;
    memcpy(pkt.data, data, len);

    // Only transmit the header + the actual payload, not the full 248-byte array.
    const size_t send_len = offsetof(CorrectionPacket, data) + len;
    return ::esp_now_send(s_peer_mac,
                          reinterpret_cast<const uint8_t*>(&pkt),
                          send_len);
}

esp_err_t send_gps_position(const GpsData& position)
{
    if (!s_peer_known) {
        ESP_LOGE(TAG, "send_gps_position: no peer known yet");
        return ESP_ERR_INVALID_STATE;
    }

    GpsPacket pkt;
    pkt.type     = PacketType::GPS_POSITION;
    pkt.position = position;

    return ::esp_now_send(s_peer_mac,
                          reinterpret_cast<const uint8_t*>(&pkt),
                          sizeof(GpsPacket));
}

// ---------------------------------------------------------------------------
// Public API — Callback registration
// ---------------------------------------------------------------------------

void set_correction_callback(CorrectionCallback cb)
{
    s_correction_cb = cb;
}

void set_gps_callback(GpsCallback cb)
{
    s_gps_cb = cb;
}

} // namespace WIFI
