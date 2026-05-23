# Connection status
bool    mavlink_connected   # true if any MAVLink received within 2 s
bool    wifi_connected      # true if WiFi peer is currently known
bool    camera_streaming    # true if camera stream was requested and not yet stopped

# MAVLink throughput
float32 radio_tx_kbps       # GCS→UAV MAVLink via radio (kbit/s)
float32 mavlink_rx_kbps     # total MAVLink received from UAV, radio + WiFi combined (kbit/s)
float32 radio_rx_kbps       # MAVLink received via radio only (kbit/s)
float32 wifi_rx_kbps        # MAVLink received via WiFi only (kbit/s)
uint32  radio_rx_msgs       # unique MAVLink messages first seen on radio (per interval)
uint32  wifi_rx_msgs        # unique MAVLink messages first seen on WiFi (per interval)
float32 uav_rx_kbps         # UAV-reported receive rate from GCS (kbit/s)

# Camera UDP throughput (WiFi-only, port 5600)
float32 camera_rx_kbps      # camera JPEG frames received at GCS (kbit/s)

# GCS-side SiK radio hardware (RADIO_STATUS, msg 109).
# 255 in rssi/noise fields means no radio or value invalid.
bool    radio_ok            # true if RADIO_STATUS received within 2 s
uint8   radio_rssi          # local signal strength [0-254]
uint8   radio_remrssi       # remote signal strength [0-254]
uint8   radio_noise         # local noise level (same units as rssi; SNR = radio_rssi - radio_noise)
uint8   radio_remnoise      # remote noise level
uint8   radio_txbuf         # TX buffer remaining [%]
uint16  radio_rxerrors      # receive error count (cumulative)
uint16  radio_fixed         # error-corrected packet count (cumulative)

# Signal quality derived from GCS local SNR (radio_rssi - radio_noise, raw SiK units ~1.9/dB).
uint8 QUALITY_NO_RADIO=0    # no RADIO_STATUS received
uint8 QUALITY_CRITICAL=1    # SNR < 20  (~10 dB) — link likely dropping packets
uint8 QUALITY_POOR=2        # SNR 20-39 (~10-20 dB) — marginal, expect errors at range
uint8 QUALITY_FAIR=3        # SNR 40-69 (~21-36 dB) — usable
uint8 QUALITY_GOOD=4        # SNR 70-99 (~37-52 dB) — solid link
uint8 QUALITY_EXCELLENT=5   # SNR ≥ 100 (~53 dB+)   — full signal
uint8   signal_quality

# UAV-side SiK radio hardware forwarded via NAMED_VALUE_FLOAT.
# 255 in rssi/noise fields means not yet received.
bool    uav_radio_ok        # true if UAV radio stats arrived within 2 s
uint8   uav_rssi            # UAV radio local RSSI
uint8   uav_noise           # UAV radio local noise (SNR = uav_rssi - uav_noise)
uint8   uav_txbuf           # UAV radio TX buffer remaining [%] — 0 = saturated
uint16  uav_rxerrors        # UAV radio RX error count (cumulative)
