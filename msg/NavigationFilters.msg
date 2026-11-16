# Jimmy Paputto 2026
# Receiver navigation filters (CFG-NAVSPG). A field is applied only when its
# has_* flag is true; unset fields keep the receiver default. Ranges are
# validated by gnsshat when the HAT starts.

bool has_min_svs
uint8 min_svs                # 3-32 satellites
bool has_max_svs
uint8 max_svs                # 3-32 satellites, >= min_svs
bool has_min_cno_dbhz
uint8 min_cno_dbhz           # 0-63 dBHz
bool has_min_elev_deg
int8 min_elev_deg            # -90..90 deg
bool has_n_cno_thrs
uint8 n_cno_thrs             # satellites required above cno_thrs_dbhz
bool has_cno_thrs_dbhz
uint8 cno_thrs_dbhz          # 0-63 dBHz
bool has_fix_mode
uint8 fix_mode               # 1=2D only, 2=3D only, 3=Auto
bool has_pdop_mask_x10
uint16 pdop_mask_x10         # PDOP mask in 0.1 units, 0-9999
bool has_tdop_mask_x10
uint16 tdop_mask_x10         # TDOP mask in 0.1 units, 0-9999
bool has_p_acc_mask_m
uint16 p_acc_mask_m          # position accuracy mask [m]
bool has_t_acc_mask_m
uint16 t_acc_mask_m          # time accuracy mask [m]
