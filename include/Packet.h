// packet.h
// Created 4-10-2018
// Contributors: Conor Hayes, Skye McEowen, Ian Heidenberger, Jamie Smith
// About: Defining the various data packets to be collected from the sensors
// 				and written to the flash memory.

#ifndef PACKET_H
#define PACKET_H

#include <mbed.h>
#include <inttypes.h>

#include <FlashLogConfig.h>

/*
 * IMPORTANT: How to add a new packet
 * ----------------------------------------------
 * 1. Add a new type ID in the definition list below with the next consecutive value.
 * 2. Update LOG_END_TYPEID to be one higher than the new largest type ID.
 * 3. Add a new struct for your packet below.  Make sure to make the last member "struct packet_tail tail".
 * 4. Add a new PRINT_LOG_<name> macro for your packet below.
 * 5. Add a new case for your packet in the switch statement in FlashLog::getPacketLen()
 * 6. Add a new case which calls your print macro in test_chip_iterate_packets() in the FlashLog test suite.
 */

//Log data types
#define LOG_DONT_DEFINE_0 0x00 //placeholder for defining 0. logic in FlashLog depends on the first actual packet type being 1
#define LOG_TEXT 0x01
#define LOG_TEMP 0x02
#define LOG_ACCEL 0x03
#define LOG_BNO 0x04
#define LOG_GPS 0x05
#define LOG_BARO 0x06
#define LOG_INVALID 0x07
#define LOG_POWER 0x08
#define LOG_ADIS  0x09
#define LOG_END_TYPEID 0xA


#define LOG_PACKET_MAGIC1 0xAAAA
#define LOG_PACKET_MAGIC2 0xAA
#define LOG_PACKET_MAGIC3 0xCCCCCCCC

#define LOG_PACKET_TEXT_LEN 64

#define MAX_PACKET_LEN sizeof(log_packet_text)

typedef uint64_t ptimer_t;		//may eventually decide to shorten this is we ever get short on room
typedef uint32_t checksum_t;

/* 	Struct to contain packet metadata
		Size: 32 bytes with padding. 29 bytes without padding--this is super wasteful, 
		but can't be helped if we insist upon 64 bit timers. Later revisions may reduce the
		number of bits allocated to the timers to stop this nonsense.
*/
/*
Each packet of data that is stored in the flash chip has a section at the end containing
some metadata, called the "packet tail." It includes some "magic numbers" which are just
constants we use to identify if a given buffer matches the format of a tail,
as well as the type of the packet, the state of the FSM when recorded, some timers, etc.
You can see its structure as well as how its used in this file
*/

struct packet_tail {
	uint16_t magic1;			/* Set to 0xAAAA to help identify the packet tail */
	uint8_t magic2;				/* Set to 0xAA, ditto */
	uint8_t state;					/* What state is our FSM in? Ascent, Recovery, etc. */
	uint8_t typeID;				/* the type of this packet. One of above LOG macros */
	/* 24 bits of padding are added here by the compiler */
	ptimer_t pwr_ctr;			/* Time since poweron */
	ptimer_t flight_ctr;		/* Time since ascent began */
	checksum_t checksum;		/* 32 bit checksum*/
	uint32_t magic3;			/* Close out the tail with a third magic packet to help us confirm the packet was fully written*/
};

/* DEBUG MACROS */
//neatly prints a packet's packet_tail struct with its values in order
#define PRINT_TAIL(PACKET_TAIL) \
        pc.printf("tail.magic = %04" PRIX16 "%02" PRIX8 "; \r\ntail.state = %s;   \
           \r\ntail.typeID = %" PRIX8 "; \r\ntail.pwr_ctr = %" PRIu64 ";    \
           \r\ntail.flight_ctr = %" PRIu64 "; \r\ntail.checksum = %08" PRIX32 "   \
           \r\ntail.magic3 = %08" PRIX32 "\r\n", \
            (PACKET_TAIL).magic1, (PACKET_TAIL).magic2, \
            FL_GET_STATE_NAME((PACKET_TAIL).state), (PACKET_TAIL).typeID, \
            (PACKET_TAIL).pwr_ctr, (PACKET_TAIL).flight_ctr, \
            (PACKET_TAIL).checksum, (PACKET_TAIL).magic3)

//prints all bytes in a packet given its type and a pointer to it/its bytes
#define PRINT_PACKET_BYTES(TYPE, PACKET_BUF)	\
    		pc.printf("[P:]"); \
				for (size_t i_printpbytes=0; i_printpbytes<getPacketLen(TYPE)-sizeof(struct packet_tail); i_printpbytes++) \
					pc.printf("%02" PRIX8, ((uint8_t*)PACKET_BUF)[i_printpbytes]); \
				pc.printf("[T:]"); \
				for (size_t i_printpbytes=getPacketLen(TYPE)-sizeof(struct packet_tail); i_printpbytes<getPacketLen(TYPE); i_printpbytes++) \
					pc.printf("%02" PRIX8, ((uint8_t*)PACKET_BUF)[i_printpbytes]); \
				pc.printf("\r\n")

//prints LEN bytes from BUF in hex
#define PRINT_BYTES(BUF, LEN) \
        for (size_t i_printbytes=0; i_printbytes<static_cast<size_t>(LEN); i_printbytes++) \
        	pc.printf("%02" PRIX8, ((char*)BUF)[i_printbytes]); \
        pc.printf("\r\n")

//neatly prints the contents of BUF in hex, divided into 64-bit chunks.
#define PRETTYPRINT_BYTES(BUF, LEN, START_ADDR) \
        for (size_t i_ppb=0; i_ppb<static_cast<size_t>(LEN); i_ppb++) { \
        	if (i_ppb%16 == 0) pc.printf("\r\n[%08" PRIX32 "]: ", static_cast<uint32_t>(i_ppb)+static_cast<uint32_t>(START_ADDR)); \
        	else if (i_ppb%4 == 0) pc.printf("-"); \
        	pc.printf("%02" PRIX8, ((char*)BUF)[i_ppb]); \
        } \
        pc.printf("\r\n")

#define PRINT_LOG_TEXT(LPTXT) \
        pc.printf("msg=%s\r\n", \
        	LPTXT->msg); \
        PRINT_TAIL(LPTXT->tail)

#define PRINT_LOG_TEMP(LPTMP) \
        pc.printf("temp=%f\r\n", \
        	LPTMP->temp); \
        PRINT_TAIL(LPTMP->tail) 

#define PRINT_LOG_ACCEL(LPA) \
        pc.printf("ax=%" PRIi16 "\tay=%" PRIi16 "\taz=%" PRIi16 "\r\n", \
        	LPA->ax, LPA->ay, LPA->az); \
        PRINT_TAIL(LPA->tail)

#define PRINT_LOG_GPS(LPG) \
        pc.printf("fixQual=%" PRIu8 "\tnumSats=%d\tlat=%f\tlong=%f\talt=%f" \
                  "\tvelNED=[%.02f, %.02f, %.02f]"\
        		  "\ttime=%" PRIu8 "/%" PRIu8 "/%" PRIu8 " %" PRIu8 ":%" PRIu8 ":%" PRIu8 "\r\n", \
        	static_cast<uint8_t>(LPG->fixQuality), LPG->numSatellites, LPG->latitude, LPG->longitude, LPG->height, \
        	LPG->northVel/100.0, LPG->eastVel/100.0, LPG->downVel/100.0, \
        	LPG->month, LPG->day, LPG->year, LPG->hour, LPG->minute, LPG->second); \
        PRINT_TAIL(LPG->tail)

#define PRINT_LOG_BNO(LPI) \
        pc.printf("qx=%f\tqy=%f\tqz=%f\tqw=%f\tax=%f\tay=%f\taz=%f\tcalib=%hhu\r\n", \
        	LPI->quatX, LPI->quatY, LPI->quatZ, LPI->quatW, LPI->ax, LPI->ay, LPI->az, LPI->calib); \
        PRINT_TAIL(LPI->tail)
        
#define PRINT_LOG_BARO(LPB) \
        pc.printf("altitude=%.02f\tpressure=%" PRIu32 "\ttemp=%.01f\r\n", \
        	LPB->altitude, LPB->pressure, LPB->temperature); \
        PRINT_TAIL(LPB->tail)

#define PRINT_LOG_POWER(power) \
        pc.printf("battVoltage=%.02f\tbattCurrent=%.03f\treg5VCurrent=%.02f\tchargePercent=%" PRIu8 "\r\n", power->battVoltage, power->battCurrent, power->reg5VCurrent, power->chargePercent);\
	PRINT_TAIL(power->tail)

#define PRINT_LOG_ADIS(adis) \
        pc.printf("gyroX=%.02f\tgyroY=%.02f\tgyroZ=%.02f\r\naccelX=%.02f\taccelY=%.02f\taccelZ=%.02f\r\n",\
		 adis->gyroX, adis->gyroY, adis->gyroZ, adis->accelX, adis->accelY, adis->accelZ);\
	PRINT_TAIL(adis->tail)

/* 	Logs text-based info messages.
		Data Size: 64 bytes
		The actual message written can be smaller than 64 bytes--just close with a null terminator
		It can also exceed 64 bytes by using multiple text packets. */

struct log_packet_text {
	char msg[LOG_PACKET_TEXT_LEN];
	struct packet_tail tail;
};

/*	Logs GPS packets 
		Data Size: TBD bytes
*/
struct log_packet_gps
{
	// latitude and logitude in decimal degrees
	double latitude;
	double longitude;

	// Height ASL in meters
	float height;

	// Quality of the current fix
	uint8_t fixQuality;

	// Estimate of the accuracy of the current position in meters
	float posAccuracy;

	// current number of sats used for navigation
	uint8_t numSatellites;

	// Variables for Velocity Solution in NED, units: cm/s
	int32_t northVel;
	int32_t eastVel;
	int32_t downVel;

	// time of the last GPS update
	uint16_t year;
	uint8_t month, day, hour, minute, second;

	struct packet_tail tail;
};

/*	Logs IMU packets 
		Data Size: 42 bytes. Pads to 44.
*/
struct log_packet_bno
{
	// quaternion rotation in radians
	float quatX, quatY, quatZ, quatW;

	// acceleration in X, Y, and Z axes in m/s^2
	float ax, ay, az;

	// magnetic field in X, Y, and Z axes in uTesla
	float magX, magY, magZ;

	char calib;

	//stability readout
	uint8_t stability;
	/* 2 bytes pad waste*/

	struct packet_tail tail;
};

/*	Logs Temperature sensor packets 
		Data Size: 4 bytes
*/
struct log_packet_temp {
	float temp;
	struct packet_tail tail;
};

/*	Logs Barometric Pressure Sensor packets 
		Data Size: TBD bytes
*/
struct log_packet_baro {
	float altitude;				/* altitude calculated from baro pressure*/
	uint32_t pressure;          // air pressure in pascals
	float temperature;          // temperature of sensor in celsius
	struct packet_tail tail;
};

/*	Logs Accelerometer packets 
		Data Size: 6 bytes. Pads to 8.
*/
struct log_packet_accel {
	int16_t ax, ay, az;
	/* 2 byte pad waste */
	struct packet_tail tail;
};

/*	Logs Power packets
	Data Size: 16 bytes
*/
struct log_packet_power {
	float battVoltage;    // voltage of battery in V
	uint8_t chargePercent; // charge percentage of battery as an integer from 0-100
	/* 3 bytes padding */
	float battCurrent; // total current used from battery in A
	float reg5VCurrent; // current used on the 5V regulated bus in A
	struct packet_tail tail;
};

/*	Logs ADIS16467 packets
 * 	Data Size: 44 bytes
 */
struct log_packet_adis {
	float    gyroX;    // 4 bytes
	float    gyroY;    // 4 bytes
	float    gyroZ;    // 4 bytes
	float    accelX;   // 4 bytes
	float    accelY;   // 4 bytes
	float    accelZ;   // 4 bytes
	float    temp;     // 4 bytes
	uint16_t dataCnt;  // 2 bytes
	//need 2 pads to fill 4 bytes
	struct packet_tail tail;
};



/*	Logs nothing--exists to pad the end of a block
		Size: 1 byte
*/
struct log_packet_empty {
	char pad;						/* Empty pad to align packets across flash banks */
};

/**
 * @brief      256-byte frame for dumping raw binary data from the log.
 */
struct log_binary_dump_frame {
	char bytes[1024*3]; 
};

#endif