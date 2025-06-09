#define PACKET_HEADER         0xFE

// Packet types from PC to TivaC
#define PKT_SENSOR_DATA       0x01    // Sensor data packet

// Packet types from TivaC to PC
#define PKT_CONTROL_CONTROL   0x02    // Vehicle control command

#define PACKET_SIZE            32
#define SENSORS_COUNT          12

// Control commands 
#define CONTROL_STOP          0x00
#define CONTROL_FORWARD       0x01
#define CONTROL_REVERSE       0x02
#define CONTROL_STEER_LEFT    0x03
#define CONTROL_STEER_RIGHT   0x04
#define CONTROL_FORWARD_LEFT  0x05
#define CONTROL_FORWARD_RIGHT 0x06
#define CONTROL_REVERSE_LEFT  0x07
#define CONTROL_REVERSE_RIGHT 0x08
#define CONTROL_BACKWARD      0x09
