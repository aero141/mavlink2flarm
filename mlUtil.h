// Patrick Eaton

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MLUTIL_H
#define MLUTIL_H

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

//  MAVLink Messages
//  Message ID  Description         I/O Length  CRC Extra
//  29          Scaled Pressure     In  14      115
//  66          DataStream Request  Out 6       148
//  201         Static              In  19      126
//  202         Ownship             Out 42      7
//  202         Dynamic             In  42      7
//  202         Navigation          In  51      11
//  203         Status              Out 1       85
//  246         Traffic Report      Out 38      184
//  248         Identification      In  69      8

// msgId, pldLength, crcExtra
//  const struct sDataStream_t sDataStream[8] =
//  {
//      {66,6,148},
//      {246,38,184},
//      {203,1,85},
//      {202,42,7},
//      {202,51,11},
//      {29,14,115},
//      {201,19,126},
//      {248,69,8}
//  };


enum eSTATE_t
{
	SYNC,
	HEADER,
	DATA
};

enum msgType_t
{
	SCALED_PRESSURE = 29,
	DATA_STREAM_REQUEST = 66,
	STATIC = 201,
	OWNSHIP = 202,
	STATUS = 203,
	TRAFFIC_REPORT = 246,
	IDENTIFICATION = 248
};


struct sScaledPressure_t
{
	uint32_t time_boot_ms;
	float press_abs;
	float press_diff;
	uint16_t temperature;
};


struct sStatic_t
{
	uint8_t ICAO[3];
	uint8_t integrity;
	uint16_t stallSpeed;
	char Callsign[8];
	uint8_t capability;
	uint8_t emitter;
	uint8_t alwEncode;
	uint8_t gpsLatOffs;
	uint8_t gpsLonOffs;
};


  // Mavlink Static Message 201
struct sDataStreamRequest_t
{
	uint8_t payLoad[6];
};


// PFLAA - Data on other procimate aircraft
// PFLAA,<AlarmLevel>,<RelativeNorth>,<RelativeEast>,
//       <RelativeVertical>,<IDType>,<ID>,<Track>,<TurnRate>,<GroundSpeed>,
//       <ClimbRate>,<AcftType>[,<NoTrack>[,<Source>,<RSSI>]]
struct sTarget_t
{
	uint32_t utcTime;  //  0 seconds since ?
	uint32_t icao;     //  4 24 bit icao address
	float relN;        //  8 meters
	float relE;        // 12 meters
	float relAlt;      // 16 meters
	float track;       // 20 degrees 0..359
	float Vg;          // 24 ground speed m/s 0 on ground
	float Vv;          // 28 -32.7 to 32.7 m/s +is up
	uint16_t alrmLvl;  // 40
	uint16_t acType;   // 42
	uint8_t tslc;      // 44
	char callsign[9];  // 45
};


// MavLink Traffic Report Message ID 246
struct sTrafficReport_t
{
	uint32_t icao;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	uint16_t heading;
	uint16_t horVel;
	int16_t verVel;
	uint16_t flags;
	uint16_t squawk;
	uint8_t altType;
	char callsign[9];
	uint8_t emitterType;
	uint8_t tslc;
};


// MavLink Ownship Message ID 202
struct sOwnShip_t
{
	uint32_t utcTime;   // 0
	int32_t lat;        // 4
	int32_t lon;        // 8
	int32_t alt;        // 12
	int32_t altGNSS;    // 16
	uint32_t accHoriz;  // 20
	uint16_t accVert;   // 24
	uint16_t accVel;    // 26
	int16_t velVert;    // 28
	int16_t nsVog;      // 30
	int16_t ewVog;      // 32
	uint16_t state;     // 34
	uint16_t squawk;    // 36
	uint8_t fixType;    // 38
	uint8_t numSats;    // 39
	uint8_t emStatus;   // 40
	uint8_t control;    // 41  42 bytes total 0..41
};


// MavLink NavData Message ID 202
struct sNavData_t
{
	uint32_t utcTime;
	int32_t lat;
	int32_t lon;
	int32_t altHAE;
	int32_t altPress;
	uint32_t horzProt;
	uint32_t vertProt;
	uint32_t horzFOM;
	uint16_t vertFOM;
	uint16_t hVelFOM;
	uint16_t vVelFOM;
	int16_t vVelGNSS;
	int16_t nsVel;
	int16_t ewVel;
	uint8_t utcFrac;
	uint8_t fixType;
	uint8_t navState;
	uint8_t satsUsed;
	uint8_t fwMajor;
	uint8_t fwMinor;
	uint8_t fwBuild;
};


struct sIdentification_t
{
	uint16_t message_type;
	uint8_t target_network;
	uint8_t target_system;
	uint8_t	target_component;
	uint8_t	primaryMajorVersion;
	uint8_t	primaryMinorVersion;
	uint8_t	primaryBuildVersion;
	uint8_t	primaryFwId;
	uint8_t	primaryHwId;
	uint64_t primarySerialNumber;
	uint32_t primaryCRC;
	uint8_t	primaryFwPartNumber[15];
	uint8_t	secondaryMajorVersion;
	uint8_t	secondaryMinorVersion;
	uint8_t	secondaryBuildVersion;
	uint8_t	secondaryFwId;
	uint8_t	secondaryHwId;
	uint64_t secondarySerialNumber;
	uint32_t secondaryCRC;
	uint8_t	secondaryFwPartNumber[15];
};


// MavLink Status Message ID 203
#define INITIALIZING 0
#define OK 1
#define TX_FAIL_1090ES 2
#define RX_FAIL_1090ES 4
#define TX_FAIL_UAT 8
#define RX_FAIL_UAT 16
struct sStatus_t
{
    uint8_t status;
};


//altitudeType
//0x00: PRESSURE_ALTITUDE (AMSL, QNH)
//0x01: GEOMETRIC (GNSS, WGS84)
//emitterType
//0x00: NO_TYPE_INFO
//0x01: LIGHT_TYPE
//0x02: SMALL_TYPE
//0x03: LARGE_TYPE
//0x04: HIGH_VORTEX_LARGE_TYPE
//0x05: HEAVY_TYPE
//0x06: HIGHLY_MANUV_TYPE
//0x07: ROTOCRAFT_TYPE
//0x08: UNASSIGNED_TYPE
//0x09: GLIDER_TYPE
//0x0A: LIGHTER_AIR_TYPE
//0x0B: PARACHUTE_TYPE
//0x0C: ULTRA_LIGHT_TYPE
//0x0D: UNASSIGNED2_TYPE
//0x0E: UAV_TYPE
//0x0F: SPACE_TYPE
//0x10: UNASSGINED3_TYPE
//0x11: EMERGENCY_SURFACE_TYPE
//0x12: SERVICE_SURFACE_TYPE
//0x13: POINT_OBSTACLE_TYPE
//flags
//0x0001: LATLON_VALID
//0x0002: ALTITUDE_VALID
//0x0004: HEADING_VALID
//0x0008: VELOCITY_VALID
//0x0010: CALLSIGN_VALID
//0x0020: IDENT_VALID
//0x0040 SIMULATED_REPORT
//0x0080 VERTICAL_VELOCITY_VALID
//0x0100 BARO_VALID
//0x8000: SOURCE UAT

// MavLink Status Message ID 203
#define INITIALIZING 0
#define OK 1
#define TX_FAIL_1090ES 2
#define RX_FAIL_1090ES 4
#define TX_FAIL_UAT 8
#define RX_FAIL_UAT 16

// msgId, pldLength, crcExtra
//  const struct sDataStream_t sDataStream[8] =
//  {
//      {66,6,148},
//      {246,38,184},
//      {203,1,85},
//      {202,42,7},
//      {202,51,11},
//      {29,14,115},
//      {201,19,126},
//      {248,69,8}
//  };


// function prototypes
void brgDstAlt(struct sTrafficReport_t* pTraffic, struct sTarget_t* target);
int  cksum (uint8_t* msg);
void clrscr(void);
void moveto(int row, int col);
void printBuffer(uint8_t* pMlMsg, int length);
void prtOwnShip (void);
void prtPFLAU(int index);
void prtPFLAA(int index);
void prtTraffic(int index);
void s2print(char*);

#endif /* mlUtil.h */

#ifdef __cplusplus
}
#endif
