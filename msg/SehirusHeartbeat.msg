# Do not use tab to define messages

# Sehirus Heartbeat msg

# Identifier ID constant 0x01AB0101
uint32 msgIdentifier

string serverName

# Status of the command listening
# port. 0=Not connected 1=Connected
uint8 commandListen

# Client connection.
# 0=Not connected 1=Connected
uint8 commandConnected

# Command listening port
uint16 commandPort

# Bandwith in MB/s
float32 rawDistributionThroughput
float32 proDistributionThroghput
float32 rawRecordingThroughput

# 0=Stop 1=Started
uint8 sourceStarted

# 0=Not full 1=Full
uint8 bufferFull

# Cpu Load 0-1000 0.1% precission
uint16 cpuLoad

# Antena period in ms
uint16 currentSourcePeriod

# currentSourcePRF

# number of antenna scans from last source start
uint32 nScans

# Navigation Data Present
# 0=No data
# 1= Data receiving set for all
# Position, speed over ground
# course, heading
uint8 navDataPresent

# Radar Latitude [-90.0-90.0] Degrees
float32 latitude

# Radar Longitude [-180.0-180.0] Degrees
float32 longitude