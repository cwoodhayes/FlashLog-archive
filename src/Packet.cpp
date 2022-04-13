//
// File containing functions for basic packet operations.
//

#include <Packet.h>

size_t getPacketLen(uint8_t type) {
	switch (type) {
		case LOG_TEXT:         	return sizeof(struct log_packet_text);
		case LOG_TEMP:         	return sizeof(struct log_packet_temp);
		case LOG_ACCEL:     	return sizeof(struct log_packet_accel);
		case LOG_BNO:         	return sizeof(struct log_packet_bno);
		case LOG_GPS:         	return sizeof(struct log_packet_gps);
		case LOG_BARO:         	return sizeof(struct log_packet_baro);
		case LOG_POWER:        	return sizeof(struct log_packet_power);
		case LOG_ADIS:        	return sizeof(struct log_packet_adis);
		case LOG_RANGEFINDER: 	return sizeof(struct log_packet_rangefinder);
		case LOG_BQZ:           return sizeof(struct log_packet_bqz);
		default:
			return 0;
	}
}

const char* getPacketName(uint8_t type)
{
	switch (type) {
		case LOG_TEXT:         return "TEXT";
		case LOG_TEMP:         return "TEMP";
		case LOG_ACCEL:        return "ACCEL";
		case LOG_BNO:          return "BNO";
		case LOG_GPS:          return "GPS";
		case LOG_BARO:         return "BARO";
		case LOG_POWER:        return "POWER";
		case LOG_ADIS:         return "ADIS";
		case LOG_RANGEFINDER:  return "RANGEFINDER";
		case LOG_BQZ:          return "BQZ";
        default:               return "Unknown Packet";
	}
}

void printPacketTail(struct packet_tail const * tail)
{
	char const * stateName;
	if(FlashLogConfig::isValidState(static_cast<FlashLogConfig::State_t>(tail->state)))
	{
        stateName = FlashLogConfig::getStateName(static_cast<FlashLogConfig::State_t>(tail->state));
	}
	else
	{
		stateName = "<invalid state>";
	}

	printf("tail.magic = %04" PRIX16 "%02" PRIX8 "; \r\ntail.state = %s;\r\n", tail->magic1, tail->magic2,  stateName);
	printf("tail.typeID = %" PRIX8 "; \r\ntail.pwr_ctr = %" PRIu64 ";\r\n", tail->typeID, tail->pwr_ctr);
	printf("tail.flight_ctr = %" PRIu64 "; \r\ntail.checksum = %08" PRIX32 "\r\n", tail->flight_ctr, tail->checksum);
	printf("tail.magic3 = %08" PRIX32 "\r\n", tail->magic3);
}

void printPacket(void const * packetBytes, uint8_t packetType)
{
	switch(packetType)
	{
		case LOG_TEXT:
		{
			auto lptxt = reinterpret_cast<struct log_packet_text const *>(packetBytes);
			printf("msg=%s\r\n", lptxt->msg);
        	printPacketTail(&lptxt->tail);
		}
		break;
		case LOG_TEMP:
		{
			auto lptmp = reinterpret_cast<struct log_packet_temp const *>(packetBytes);
			printf("temp=%f\r\n", lptmp->temp);
			printPacketTail(&lptmp->tail);
		}
		break;
		case LOG_ACCEL:
		{
			auto lpaccel = reinterpret_cast<struct log_packet_accel const *>(packetBytes);
			printf("ax=%" PRIi16 "\tay=%" PRIi16 "\taz=%" PRIi16 "\r\n", lpaccel->ax, lpaccel->ay, lpaccel->az);
			printPacketTail(&lpaccel->tail);
		}
		break;
		case LOG_GPS:
		{
			auto lpgps = reinterpret_cast<struct log_packet_gps const *>(packetBytes);
            printf("fixQual=%" PRIu8 "\tnumSats=%d\tlat=%f\tlong=%f\talt=%" PRIi32
                      "\tvelNED=[%.02f, %.02f, %.02f]"
                      "\ttime=%" PRIu8 "/%" PRIu8 "/%" PRIu16 " %" PRIu8 ":%" PRIu8 ":%" PRIu8
                      "\r\n",
                static_cast<uint8_t>(lpgps->fixQuality),
                lpgps->numSatellites,
                lpgps->latitude,
                lpgps->longitude,
                lpgps->height,
                lpgps->northVel / 100.0,
                lpgps->eastVel / 100.0,
                lpgps->downVel / 100.0,
                lpgps->month,
                lpgps->day,
                lpgps->year,
                lpgps->hour,
                lpgps->minute,
                lpgps->second);
            printPacketTail(&lpgps->tail);
        }
		break;
		case LOG_BNO:
		{
			auto lpbno = reinterpret_cast<struct log_packet_bno const *>(packetBytes);
			printf("qx=%f\tqy=%f\tqz=%f\tqw=%f\tax=%f\tay=%f\taz=%f\tcalib=%hhu\r\n",
        		lpbno->quatX, lpbno->quatY, lpbno->quatZ, lpbno->quatW, lpbno->ax, lpbno->ay, lpbno->az, lpbno->calib);
        	printPacketTail(&lpbno->tail);
		}
		break;
		case LOG_BARO:
		{
			auto lpbaro = reinterpret_cast<struct log_packet_baro const *>(packetBytes);
			printf("altitude=%.02f\tpressure=%" PRIu32 "\ttemp=%.01f\r\n",
        		lpbaro->altitude, lpbaro->pressure, lpbaro->temperature);
			printPacketTail(&lpbaro->tail);
		}
		break;
		case LOG_POWER:
		{
			auto lppower = reinterpret_cast<struct log_packet_power const *>(packetBytes);
			printf("current3V=%.03f\tcurrent5V=%.03f\tvoltage3V=%.02f\tvoltage5V=%.02f\r\n",
 				lppower->current3V,
 				lppower->current5V,
 				lppower->voltage3V,
 				lppower->voltage5V);
			printPacketTail(&lppower->tail);
		}
		break;
		case LOG_ADIS:
		{
			auto lpadis = reinterpret_cast<struct log_packet_adis const *>(packetBytes);
			printf("gyroX=%.02f\tgyroY=%.02f\tgyroZ=%.02f\r\naccelX=%.02f\taccelY=%.02f\taccelZ=%.02f\r\n",
			 	lpadis->gyroX, lpadis->gyroY, lpadis->gyroZ, lpadis->accelX, lpadis->accelY, lpadis->accelZ);
			printPacketTail(&lpadis->tail);
		}
		break;
		case LOG_RANGEFINDER:
		{
			auto lprange = reinterpret_cast<struct log_packet_rangefinder const *>(packetBytes);
			printf("distance=%.02f\trssi=%.01f\tlqi=%" PRIi8 "\r\n",
				lprange->distance, lprange->rssi, lprange->linkQual);
			printPacketTail(&lprange->tail);
		}
		break;
		case LOG_BQZ:
		{
			auto lpbqz = reinterpret_cast<struct log_packet_bqz const *>(packetBytes);
			printf("charge=%d%%\r\n",
				bqz->charge);
			printPacketTail(&lpbqz->tail);
		}
		break;
		default:
			printf("<Unknown packet>\r\n");
	}
}
