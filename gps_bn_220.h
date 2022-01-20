#ifndef GPS_BN_220
#define GPS_BN_220

#include "main.h"


/* ----------
*
* Typedefs
*
---------- */ 

// The Equator has a latitude of 0°,
//the North Pole has a latitude of 90° North (written 90° N or +90°),
//and the South Pole has a latitude of 90° South (written 90° S or −90°)
// longitude in degrees (0° at the Prime Meridian to +180° eastward and −180° westward)
// that is why 3

typedef struct{
	// Flags
	uint8_t flag;
	uint8_t flag2;
	// Buffers
	uint8_t buff[255];
	char buffStr[255];
	char nmeaSnt[80];
	// 
	char *rawSum;
	char smNmbr[3];
	// Latitude (N / + and S / -)
	char *latRaw;
	char latDg[2];
	char latMs[7];
	char *hemNS;
	// Longitude (W / + and E / -)
	char *lonRaw;
	char lonDg[3];
	char lonMs[7];
	char *hemEW;
	// Time
	char *utcRaw;
	char strUTC[8];
	char hH[2];
	char mM[2];
	char sS[2];
	// Counter
	uint8_t cnt;
	// Interface
	UART_HandleTypeDef *huart;
} GPS_DATA;

/* ----------
*
* Function prototypes
*
---------- */ 

void GPS_Init(GPS_DATA *input, UART_HandleTypeDef *handle_uart);
int16_t GPS_nmea0183_checksum(char *msg);
void GPS_DMA_Receive(GPS_DATA *input);
void GPS_Parse_Data(GPS_DATA *input);
void GPS_Parse_GNGLL(GPS_DATA *input);
void GPS_Parse_GNGGA(GPS_DATA *input);
// float GPS_Char2Float(GPS_DATA *input);


#endif
