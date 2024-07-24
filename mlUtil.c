// Patrick Eaton

#include <Arduino.h>
#include "mlUtil.h"

extern struct sOwnShip_t ownShip;
extern struct sTarget_t target[32];
extern struct sTrafficReport_t traffic[32];

// ownShip, target[32]
void brgDstAlt(struct sTrafficReport_t* pTraffic, struct sTarget_t* target)
{
	// const double Re = 3443.92;			// NM
	const float Re = 6.3781*1.0E6;			// meters
	const float d2r = 3.14159 / 180.0;
	float lat1, lon1, alt1, lat2, lon2, alt2, dLat, dLon;
	
	lat1 = (float)ownShip.lat * 1.0E-7;		// ownShip lat in degrees
	lon1 = (float)ownShip.lon * 1.0E-7;		// ownship lon in degrees
	alt1 = (float)ownShip.alt * 1.0E-3;		// ownShip alt in meters
	lat2 = (float)pTraffic->lat * 1.0E-7;	// traffic lat in degrees
	lon2 = (float)pTraffic->lon * 1.0E-7;	// traffic lon in degrees
	alt2 = (float)pTraffic->alt * 1.0E-3;	// traffic alt in meters
	dLat = (lat2 - lat1);								// degrees
	dLon = (lon2 - lon1) * cos((lat1 + lat2) / 2.0);	// adj degrees

	// target: utcTime,icao,relN,relE,relAlt,track,Vg,Vv,alrmLvl,acType,callsign
	target[0].utcTime = ownShip.utcTime;
	target[0].relN = dLat * d2r * Re;					// wRe (omega Re)
	target[0].relE = dLon * d2r * Re;
	target[0].relAlt = (alt2 - alt1);					// meters
	target[0].icao = pTraffic->icao;
	target[0].track = atan2(dLat, dLon) / d2r;		// -180.0 ... 180.0
	// dist = Re * sqrt((dLat * dLat) + (dLon * dLon));
	strncpy(target[0].callsign, pTraffic->callsign, 8);

	// printf("\r\nDEBUG %8.4f %9.4f %8.4f %9.4f\r\n", lat1, lon1, lat2, lon2);
	// if  (brg < 90.0)
	{
		// brg = 90.0 - brg;
	}
	// else
	{
		// brg = 450.0 - brg;
	}
	// printf("DEBUG %8.4f %8.4f %6.1f %6.1f\r\n", dLat, dLon, dist, brg);
	// Filter if (lat==0 and lon==0) skip
	// Filter if (dAlt > ?) skip
	// Filter if (dist > x NM) skip
	// moveto(7, 4);
	prtPFLAA(0);

	return;
}


void clrscr(void)
{
	// VT102 Clear Screen
	printf("%c[2J", 0x1b);
}


void moveto(int row, int col)
{
	// Serial.write(0x1B);
	printf("%c[%u;%uH", 0x1b, row, col);
}


void printBuffer(uint8_t* pMlMsg, int length)
{
	// Print the buffer
	for (int bIdx=0; bIdx<length; bIdx++)
	{
		printf("%02X", pMlMsg[bIdx]);
	}
	printf("\r\n");
	return;
}


// PFLAU - Heartbeat, status, and basic alarms
// PFLAU,<RX>,<TX>,<GPS>,<Power>,<AlarmLevel>,<RelativeBearing>,
//		 <AlarmType>,<RelativeVertical>,<RelativeDistance>[,<ID>]
void prtPFLAU(int index)
{
	char msg[128];
	int nRx = 3;		// No. devices with unique ID's currently received
	int alarmLevel = 2;	// 0=no alarm, 1=15-20 sec, 2=10-15 sec, 3=0-10 sec
	int relBrg = -30;	// relative bearing -180 to 180
	int alarmType = 2;	// 0=No A/C within rng, 2=A/C, 3=Obst., 4=Traffic Advisory
	int relVert = -32;	// relative vertical in meters
	int relDist = 755;	// relative distance in meters
	int ID = 0x5A77B1;	// ICAO 24 bit address

	// $PFLAU,2,1,2,1,1,0,41,0,0,A25703*
	snprintf(msg, 127, "$PFLAU,%d,1,2,1,%d,%d,%d,%d,%d,%06X*\r\n",
		nRx, alarmLevel, relBrg, alarmType, relVert, relDist, ID
		);
	s2print(msg);

	return;
}


// PFLAA - Data on other procimate aircraft
// PFLAA,<AlarmLevel>,<RelativeNorth>,<RelativeEast>,
//		 <RelativeVertical>,<IDType>,<ID>,<Track>,<TurnRate>,<GroundSpeed>,
//		 <ClimbRate>,<AcftType>[,<NoTrack>[,<Source>,<RSSI>]]
void prtPFLAA (int index)
{
	// const float D2M = 60.0 * 1852.0;    // 1 NM = 1852.0 meters, 1 degree = 60.0 NM
	// float avgLat;
	char msg[128];
	int alarmLevel = 0;
	int IdType = 1;		// 0=random ID, 1=ICAO 24 bit addr, 2=fixed FLARM ID
	// int noTrack;
	// int source;
	// int RSSI;

//	  relN = (int)((lat2 - lat1) * D2M);
//	  avgLat = (lat2 + lat1) / 2.0;
//	  relE = (int)((lon2 - lon1) * cos(avgLat) * D2M);
//	  relVert = trafficReport.alt - ownShip.alt;
	// $PFLAA,0,-1234,1234,220,2,DD8F12,180,,30,-1.4,1*
	snprintf(msg, 127, "$PFLAA,%0d,%.1f,%.1f,%.1f,%d,%06lX,%.1f,,%.1f,%0.1f,%d*",
		alarmLevel,
		target[index].relN,
		target[index].relE,
		target[index].relAlt,
		IdType,
		target[index].icao,
		target[index].track,
		target[index].Vg,
		target[index].Vv,
		target[index].acType
		);
	// cksum (msg);
	s2print(msg);
	snprintf(msg, 5, "%02X\r\n", cksum(msg));
	s2print(msg);

	return;
}


void prtTraffic(int index)
{
  char msg[128];

  snprintf(msg, 127, "Traffic: %06lX,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,0x%0X,%04d,%d,%s,%u,%u\r\n",
    traffic[index].icao,                      // %06X
	  (float)traffic[index].lat * 1.0E-7,       // %.4f
	  (float)traffic[index].lon * 1.0E-7,       // %.4f
	  (float)traffic[index].alt * 1.0E-3,       // %.2f
	  (float)traffic[index].heading * 1.0E-2,   // %.2f
	  (float)traffic[index].horVel  * 1.0E-2,   // %.2f
	  (float)traffic[index].verVel * 1.0E-2,    // %.2f
	  traffic[index].flags,                     // %0X
	  traffic[index].squawk,                    // %04d
	  traffic[index].altType,                   // %d
	  traffic[index].callsign,                  // %s
	  traffic[index].emitterType,               // %u
	  traffic[index].tslc                       // %u
  );
  s2print(msg);

  return;
}


void prtOwnShip (void)
{
	time_t mlTime;
	struct tm *pstm;
  char msg[128];

  mlTime = ownShip.utcTime;
  pstm = gmtime((const time_t*)&mlTime);

  // moveto(1,2);
  // 2024/06/08 16:56:30  N201AS    45.3077  -92.6913 278.0 326.4 1200 3
	snprintf(msg, 127, "%04d/%02d/%02d %02d:%02d:%02d  N201AS   %8.4f %9.4f %5.1f %5.1f %04u %1u\r\n",
	  pstm->tm_year+1900,
		pstm->tm_mon+1,
		pstm->tm_mday,
		pstm->tm_hour,
		pstm->tm_min,
		pstm->tm_sec,
		(float)ownShip.lat * 1E-7,
		(float)ownShip.lon * 1E-7,
		(float)ownShip.alt * 1E-3,
    (float)ownShip.altGNSS * 1E-3,
    ownShip.squawk,
		ownShip.fixType
	);
  
  printf(msg);
  s2print(msg);

  return;
}


int cksum (uint8_t* msg)
{
	int	checksum;
	int maxchar = 0;

	checksum = 0;
	if (*msg == '$') { msg++; }
	while (*msg != '*')
	{
		checksum ^= *msg++;
		maxchar++;
		if (maxchar > 83) { break; }
	}
	// xor everything between '$' and '*'
	return checksum;
}
