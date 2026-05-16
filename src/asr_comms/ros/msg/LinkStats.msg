float32 tx_kbps         # bytes/s sent over the SiK radio (GCS→UAV)
float32 rx_kbps         # total bytes/s received from UAV (both paths combined)
float32 radio_rx_kbps   # bytes/s received via SiK radio only
float32 wifi_rx_kbps    # bytes/s received via WiFi only
uint32  radio_rx_msgs   # unique MAVLink messages first seen on the radio path (per interval)
uint32  wifi_rx_msgs    # unique MAVLink messages first seen on the WiFi path (per interval)
float32 peer_rx_kbps    # bytes/s the UAV actually received (reported by UAV)
bool    connected        # true if any message was received within the last 0.5 s
bool    wifi_connected   # true if the WiFi peer is currently known

# SiK radio hardware stats (from MAVLink RADIO_STATUS, msg 109).
# 255 in rssi/noise fields means no radio or value invalid.
bool    radio_ok     # true if RADIO_STATUS was received within the last 2 s
uint8   rssi         # local signal strength [0-254]
uint8   remrssi      # remote signal strength [0-254]
uint8   noise        # local noise level (same units as rssi; SNR = rssi - noise)
uint8   remnoise     # remote noise level
uint8   txbuf        # TX buffer remaining [%]
uint16  rxerrors     # receive error count (cumulative)
uint16  fixed        # error-corrected packet count (cumulative)

# Signal quality derived from local SNR (rssi - noise, raw SiK units ~1.9/dB).
uint8 QUALITY_NO_RADIO=0   # no RADIO_STATUS received
uint8 QUALITY_CRITICAL=1   # SNR < 20  (~10 dB) — link likely dropping packets
uint8 QUALITY_POOR=2       # SNR 20-39 (~10-20 dB) — marginal, expect errors at range
uint8 QUALITY_FAIR=3       # SNR 40-69 (~21-36 dB) — usable
uint8 QUALITY_GOOD=4       # SNR 70-99 (~37-52 dB) — solid link
uint8 QUALITY_EXCELLENT=5  # SNR ≥ 100 (~53 dB+)   — full signal
uint8   signal_quality

# UAV-side radio stats forwarded from comms_uav via NAMED_VALUE_FLOAT.
# 255 in rssi/noise fields means not yet received.
bool    uav_radio_ok     # true if UAV radio stats arrived within the last 2 s
uint8   uav_txbuf        # UAV radio TX buffer remaining [%] — 0 = saturated
uint8   uav_rssi         # UAV radio local RSSI
uint8   uav_noise        # UAV radio local noise (SNR = uav_rssi - uav_noise)
uint16  uav_rxerrors     # UAV radio RX error count (cumulative)
