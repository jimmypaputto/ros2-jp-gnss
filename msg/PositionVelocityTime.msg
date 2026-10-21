# Jimmy Paputto 2026

# Fix information
uint8 fix_quality        # 0=Invalid, 1=GPS 2D/3D, 2=DGNSS, 3=PPS, 4=FixedRTK, 5=FloatRTK, 6=DeadReckoning
uint8 fix_status         # 0=Void, 1=Active
uint8 fix_type           # 0=NoFix, 1=DeadReckoningOnly, 2=2D, 3=3D, 4=GnssWithDR, 5=TimeOnly

# Position
float64 latitude         # degrees
float64 longitude        # degrees
float32 altitude         # meters above ellipsoid
float32 altitude_msl     # meters above mean sea level

# Velocity
float32 speed_over_ground  # m/s
float32 speed_accuracy     # m/s
float32 heading            # degrees
float32 heading_accuracy   # degrees

# Accuracy
float32 horizontal_accuracy  # m
float32 vertical_accuracy    # m

# Satellites
uint8 visible_satellites

# UTC Time
uint8  utc_hours
uint8  utc_minutes
uint8  utc_seconds
bool   utc_valid
int32  utc_accuracy      # ns

# Date
uint8  date_day
uint8  date_month
uint16 date_year
bool   date_valid
