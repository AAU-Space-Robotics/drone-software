float64 timestamp               # s

float32 rx_kbps                 # kbit/s received from GCS (all paths combined)
float32 gcs_msg_age_ms          # ms since any message arrived from GCS
bool    gcs_connected           # true if gcs_msg_age_ms < 1000
