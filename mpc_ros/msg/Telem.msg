# values obtained from flight controller
int32 lat         # latitude
int32 lon         # longitude
int32 alt         # altitude (MSL)
float64 vx          # ground x velocity (m/s, latitude, positive north)
float64 vy          # ground y velocity (m/s, longitude, positive east)
float64 vz          # ground z velocity (m/s, altitude, positive DOWN)
uint16 heading    # heading (0 to 359.99, if unknown set to UINT16_MAX)
float64 roll        # roll angle (-pi ... pi)
float64 pitch       # pitch angle (-pi ... pi)
float64 yaw         # yaw angle (-pi ... pi)
float64 x         # local ned x (m, positive north)
float64 y         # local ned y (m, positive east)
float64 z         # local ned z (m, positive DOWN)