#include "gps_bn_220.h"
#include "string.h"
#include "stdio.h"

void GPS_Init(GPS_DATA *input, UART_HandleTypeDef *handle_uart){
	// Fill interface data
	input->huart = handle_uart;
}


int16_t GPS_nmea0183_checksum(char *msg){
	// Calculate checksum
	int16_t gps_220_checksum = 0;

	for( int16_t j = 1; j < strlen(msg) - 4; j++ ){
		gps_220_checksum ^= msg[j];
	}
	return gps_220_checksum;
}

void GPS_DMA_Receive(GPS_DATA *input){
	// UART1 Receives data
	// Put this right before the while(1)
	//
	// To set DMA full flag: (Put in PTD)
	// 
	HAL_UART_Receive_DMA(input->huart, input->buff, 255);
}

void GPS_Receive_Cplt(GPS_DATA *input){
	// Set flag when buffer is filled
	input->flag = 1;
}


void GPS_Parse_Data(GPS_DATA *input){
	// Do the parsing in the while(1)
	
	if (input->flag == 1){ 
		// interrupt signals that the buffer buff[255] is full
	    /*

	  	  $ - Start delimiter
	  	  * - Checksum delimiter
	  	  , - Field delimiter

	  	  1. $GNGLL log header
	  	  2. Latitude (Ddmm.mm) [The Equator has a latitude of 0°, the North Pole has a latitude of 90° North (written 90° N or +90°)]
	  	  3. Latitude direction (N = North, S = South)
	  	  4. Longitude (DDDmm.mm) [0° at the Prime Meridian to +180° eastward and −180° westward]
	  	  5. Longitude direction (E = East, W = West)
	  	  6. UTC time status of position (hours/minutes/seconds/decimal seconds) hhmmss
	  	  7. Data status: A = Data valid, V = Data invalid
	  	  8. Positioning system mode indicator
	  	  9. *xx Checksum
	  	  10. [CR][LF] Sentence terminator. In C \r\n (two characters).
	  	   or \r Carriage return
	  	   or \n Line feed, end delimiter

	  	*/

	  	memset(input->buffStr, 0, 255);	// set buffStr to be array[255] of zeros

	  	sprintf(input->buffStr, "%s", input->buff);	// set buffStr to be buff

	  	// splitting the buffStr by the "\n" delimiter with the strsep() C function
	  	char *token, *string;

		string = strdup(input->buffStr); // create a string that is the address of buffStr
		// actually splitting the string by "\n" delimiter
	  	while( (token = strsep(&string, "\n")) != NULL ){
	  		memset(input->nmeaSnt, 0, 80);	// set nmeaSnt to be zeros
	  		sprintf(input->nmeaSnt, "%s", token); // buffStr split by \n

	  		if( (strstr(input->nmeaSnt, "$GNGLL") != 0) && strlen(input->nmeaSnt) > 49 && strstr(input->nmeaSnt, "*") != 0 ){
	  			GPS_Parse_GNGLL(input);
	  		}
			
			// 
	  		else if( (strstr(input->nmeaSnt, "$GNGGA") != 0) && strlen(input->nmeaSnt) > 72 && strstr(input->nmeaSnt, "*") != 0 ){
	  			GPS_Parse_GNGGA(input);
	  		}
			
			// Message succesfully parsed
	  		if( input->flag2 == 1 ){
	  			break;
	  		}
	  	}
	  	input->flag = 0;   // reset the flag

	}
	
	if(input->flag2 == 1){
		HAL_Delay(500);
		input->flag2 = 0;
	}
	else{
		HAL_Delay(200);
	}
}



void GPS_Parse_GNGLL(GPS_DATA *input){
	// if $GNGLL is present, if the string is long enough anf if the * is present, only then is the msg valid
	input->rawSum = strstr(input->nmeaSnt, "*"); // We get a pointer to the star location, checksum is found after this
	memcpy(input->smNmbr, &input->rawSum[1], 2); // Copy checksum value after star
	input->smNmbr[3] = '\0'; // save the given checksum result

	uint8_t intSum = GPS_nmea0183_checksum(input->nmeaSnt); // calculate checksum value

	char hex[2]; // variable since the checksum result is given in hex format
	sprintf(hex, "%X", intSum);	// print the result of the checksum into hex (%X unsigned hex with capital letters)

	// verify checksum data --> if it is OK the position can be trusted
	if( strstr(input->smNmbr, hex) != NULL ){

	input->cnt = 0; // initialize counter

	// split the good NMEA sentence into tokens by commas
	for( char *pV = strtok(input->nmeaSnt, ","); pV != NULL; pV = strtok(NULL, ",") ){
		switch( input->cnt ){
		case 1:
			input->latRaw = strdup(pV);
	  		break;
	  	case 2:
	  		input->hemNS = strdup(pV);
	  		break;
	  	case 3:
	  		input->lonRaw = strdup(pV);
	  		break;
	  	case 4:
	  		input->hemEW = strdup(pV);
	  		break;
	  	case 5:
	  		input->utcRaw = strdup(pV);
	  		break;
	  	}
			
	  	input->cnt++;
	  	if( input->cnt == 6 ){
	  		break;
	  	}
	} // Measured raw values are written

	memcpy(input->latDg, &input->latRaw[0], 2);
	input->latDg[2] = '\0';

	memcpy(input->latMs, &input->latRaw[2], 7);
	input->latMs[7] = '\0';

	memcpy(input->lonDg, &input->lonRaw[0], 3);
	input->lonDg[3] = '\0';

	memcpy(input->lonMs, &input->lonRaw[3], 7);
	input->lonMs[7] = '\0';
	char strLonMs[7];
	sprintf(strLonMs, "%s", input->lonMs);

	// convert UTC time to the format hh:mm:ss format
	memcpy(input->hH, &input->utcRaw[0], 2);
	input->hH[2] = '\0';

	memcpy(input->mM, &input->utcRaw[2], 2);
	input->mM[2] = '\0';

	memcpy(input->sS, &input->utcRaw[4], 2);
	input->sS[2] = '\0';

	strcpy(input->strUTC, input->hH);
	strcat(input->strUTC, ":");
	strcat(input->strUTC, input->mM);
	strcat(input->strUTC, ":");
	strcat(input->strUTC, input->sS);
	input->strUTC[8] = '\0';

	input->flag2 = 1;
	}
}


void GPS_Parse_GNGGA(GPS_DATA *input){
	// if $GNGGA is present, if the string is long enough anf if the * is present, only then is the msg valid

	input->rawSum = strstr(input->nmeaSnt, "*"); // We get a pointer to the star location, checksum is found after this
	memcpy(input->smNmbr, &input->rawSum[1], 2); // Copy checksum value after star
	input->smNmbr[3] = '\0'; // save the given checksum result

	uint8_t intSum = GPS_nmea0183_checksum(input->nmeaSnt); // calculate checksum value

	char hex[2]; // variable since the checksum result is given in hex format
	sprintf(hex, "%X", intSum);	// print the result of the checksum into hex (%X unsigned hex with capital letters)

	// verify checksum data --> if it is OK the position can be trusted
	if( strstr(input->smNmbr, hex) != NULL ){

	input->cnt = 0; // initialize counter

	// split the good NMEA sentence into tokens by commas
	for( char *pV = strtok(input->nmeaSnt, ","); pV != NULL; pV = strtok(NULL, ",") ){
	  	switch( input->cnt ){
	  	case 1:
	  			  		input->utcRaw = strdup(pV);
	  			  		break;
	  	case 2:
	  			  		input->latRaw = strdup(pV);
	  			  		break;
	  	case 3:
	  			  		input->hemNS = strdup(pV);
	  			  		break;
	  	case 4:
	  			  		input->lonRaw = strdup(pV);
	  			  		break;
	  	case 5:
	  			  		input->hemEW = strdup(pV);
	  			  		break;
	  	}

		input->cnt++;
		if( input->cnt == 6 ){
			break;
		}
	} // Measured raw values are written

	memcpy(input->latDg, &input->latRaw[0], 2);
	input->latDg[2] = '\0';

	memcpy(input->latMs, &input->latRaw[2], 7);
	input->latMs[7] = '\0';

	memcpy(input->lonDg, &input->lonRaw[0], 3);
	input->lonDg[3] = '\0';

	memcpy(input->lonMs, &input->lonRaw[3], 7);
	input->lonMs[7] = '\0';
	char strLonMs[7];
	sprintf(strLonMs, "%s", input->lonMs);

	// convert UTC time to the format hh:mm:ss format
	memcpy(input->hH, &input->utcRaw[0], 2);
	input->hH[2] = '\0';

	memcpy(input->mM, &input->utcRaw[2], 2);
	input->mM[2] = '\0';

	memcpy(input->sS, &input->utcRaw[4], 2);
	input->sS[2] = '\0';

	strcpy(input->strUTC, input->hH);
	strcat(input->strUTC, ":");
	strcat(input->strUTC, input->mM);
	strcat(input->strUTC, ":");
	strcat(input->strUTC, input->sS);
	input->strUTC[8] = '\0';

	input->flag2 = 1;
	}
}


/*
float GPS_Char2Float(GPS_DATA *input){
	return 0;
}
*/



