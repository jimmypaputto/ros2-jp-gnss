# Jimmy Paputto 2026
# Complete GNSS navigation message

std_msgs/Header header

jp_gnss_hat/PositionVelocityTime pvt
jp_gnss_hat/DilutionOverPrecision dop
jp_gnss_hat/GeofencingNav geofencing
jp_gnss_hat/RfBlock[] rf_blocks
jp_gnss_hat/SatelliteInfo[] satellites

# The receiver refreshes the spectrum about once per second, so it is set
# only in the first message after it changes and left empty otherwise.
jp_gnss_hat/RfBlockSpectrum[] rf_spectrum
