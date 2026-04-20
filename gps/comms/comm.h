#ifndef COMMS_COMM_H
#define COMMS_COMM_H

#include "esp_now.h"    // IDF ESP-NOW API (no naming conflict now that our file is comm.h)
#include "esp_err.h"
#include "structs.h"
#include <stdint.h>
#include <stddef.h>

namespace WIFI {

// ---------------------------------------------------------------------------
// Packet layout
// ---------------------------------------------------------------------------

// Every ESP-NOW frame starts with this discriminator byte.
enum class PacketType : uint8_t {
    CORRECTION   = 0x01,   // Ground station → Rover  : complete RTCM frame (fits in one packet)
    GPS_POSITION = 0x02,   // Rover           → Ground : parsed GpsData
    FRAGMENT     = 0x03,   // Ground station → Rover  : one slice of a large RTCM frame
};

// ESP-NOW hard limit is 250 bytes per frame.
// CorrectionPacket header = type(1) + len(1) = 2 bytes, leaving 248 for payload.
static constexpr uint8_t MAX_CORRECTION_LEN = 248;

// Fragment header occupies 5 bytes, leaving 245 bytes for payload data.
static constexpr uint8_t MAX_FRAGMENT_DATA  = 245;

// Sent by the ground station to the rover carrying raw RTCM correction bytes
// that fit within a single ESP-NOW frame.
struct __attribute__((packed)) CorrectionPacket {
    PacketType type;                    // = PacketType::CORRECTION
    uint8_t    len;                     // number of valid bytes in data[]
    uint8_t    data[MAX_CORRECTION_LEN];
};

// Sent when an RTCM frame is too large for one ESP-NOW packet.
// The receiver collects all fragments with the same msg_id and reassembles them.
struct __attribute__((packed)) FragmentPacket {
    PacketType type;                    // = PacketType::FRAGMENT
    uint8_t    msg_id;                  // identifies which message these fragments belong to
    uint8_t    frag_idx;                // index of this fragment (0-based)
    uint8_t    total_frags;             // total number of fragments for this message
    uint8_t    len;                     // number of valid bytes in data[]
    uint8_t    data[MAX_FRAGMENT_DATA];
};

// Sent by the rover to the ground station carrying the parsed GPS position.
struct __attribute__((packed)) GpsPacket {
    PacketType type;                    // = PacketType::GPS_POSITION
    GpsData    position;
};

// ---------------------------------------------------------------------------
// User-supplied callback types
// ---------------------------------------------------------------------------

// Called on the Rover when correction data arrives.
// data/len are the raw RTCM bytes; src_mac is the sender's MAC (6 bytes).
using CorrectionCallback = void (*)(const uint8_t* data, size_t len,
                                    const uint8_t* src_mac);

// Called on the Ground Station when a GPS position arrives.
// src_mac is the sender's MAC (6 bytes).
using GpsCallback = void (*)(const GpsData& position, const uint8_t* src_mac);

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

// Initialise WiFi in STA + Long-Range mode on channel 1.
// The caller must initialise NVS (nvs_flash_init) before calling this.
esp_err_t wifi_init();

// Initialise ESP-NOW and register internal send/receive callbacks.
// Must be called after wifi_init().
esp_err_t esp_now_init();

// ---------------------------------------------------------------------------
// Peer management
// ---------------------------------------------------------------------------

// Register a peer MAC with ESP-NOW and mark it as the send target.
// Call this if you know the remote MAC at boot; otherwise the first received
// packet's source address is used automatically (auto-learn).
esp_err_t add_peer(const uint8_t* mac_addr);

// Returns true once a send-target peer is available (via add_peer or auto-learn).
bool is_peer_known();

// Returns a pointer to the 6-byte send-target MAC, or nullptr if not yet known.
const uint8_t* get_peer_mac();

// ---------------------------------------------------------------------------
// Sending
// ---------------------------------------------------------------------------

// Send raw RTCM correction bytes to the registered peer (Ground Station → Rover).
// Frames up to MAX_CORRECTION_LEN are sent in one packet; larger frames are
// automatically split into FragmentPackets and reassembled on the rover side.
esp_err_t send_correction(const uint8_t* data, size_t len);

// Send a GPS position to the registered peer (Rover → Ground Station).
esp_err_t send_gps_position(const GpsData& position);

// ---------------------------------------------------------------------------
// Callback registration
// ---------------------------------------------------------------------------

// Register the callback invoked when correction data arrives (Rover side).
void set_correction_callback(CorrectionCallback cb);

// Register the callback invoked when GPS position data arrives (Ground Station side).
void set_gps_callback(GpsCallback cb);

} // namespace WIFI

#endif // COMMS_COMM_H
