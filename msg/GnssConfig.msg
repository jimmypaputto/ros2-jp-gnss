# Jimmy Paputto 2026
# Complete GNSS HAT configuration

uint16 measurement_rate_hz
uint8 dynamic_model
bool publish_standard_topics
bool use_ntrip_rtcm
bool save_to_flash

# Timepulse
bool timepulse_active
uint32 timepulse_frequency
float32 timepulse_pulse_width
bool timepulse_has_no_fix_pulse
uint32 timepulse_no_fix_frequency
float32 timepulse_no_fix_pulse_width
uint8 timepulse_polarity              # 0=FallingEdge, 1=RisingEdge

# Geofencing
bool geofencing_enabled
uint8 geofencing_confidence_level     # 0=none, 1=68%, 2=95%, 3=99.7%, 4=99.99%, 5=99.9999%
bool geofencing_has_pio_pin_polarity
uint8 geofencing_pio_pin_polarity     # 0=LowMeansInside, 1=LowMeansOutside
jp_gnss_hat/Geofence[] geofences

# RTK
bool rtk_enabled
uint8 rtk_mode                        # 0=Base, 1=Rover
uint8 rtk_base_type                   # 0=survey_in, 1=fixed_ecef, 2=fixed_lla
uint32 rtk_min_observation_time_s
float64 rtk_required_accuracy_m
float64 rtk_ecef_x_m
float64 rtk_ecef_y_m
float64 rtk_ecef_z_m
float64 rtk_lla_latitude_deg
float64 rtk_lla_longitude_deg
float64 rtk_lla_height_m
float64 rtk_position_accuracy_m

# Navigation filters (all has_* false = receiver defaults)
jp_gnss_hat/NavigationFilters navigation_filters
