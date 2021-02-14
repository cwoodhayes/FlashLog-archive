//
// File containing functions for basic packet operations.
//

#include <Packet.h>

size_t getPacketLen(uint8_t type) {
	switch (type) {
		case LOG_TEXT:         return sizeof(struct log_packet_text);
		case LOG_TEMP:         return sizeof(struct log_packet_temp);
		case LOG_ACCEL:     return sizeof(struct log_packet_accel);
		case LOG_BNO:         return sizeof(struct log_packet_bno);
		case LOG_GPS:         return sizeof(struct log_packet_gps);
		case LOG_BARO:         return sizeof(struct log_packet_baro);
		case LOG_POWER:        return sizeof(struct log_packet_power);
		case LOG_ADIS:        return sizeof(struct log_packet_adis);
		case LOG_RANGEFINDER: return sizeof(struct log_packet_rangefinder);
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
        default:               return "Unknown Packet";
	}

}

void printPacketTail(struct packet_tail const * tail, Stream & pc)
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

	pc.printf("tail.magic = %04" PRIX16 "%02" PRIX8 "; \r\ntail.state = %s;\r\n", tail->magic1, tail->magic2,  stateName);
	pc.printf("tail.typeID = %" PRIX8 "; \r\ntail.pwr_ctr = %" PRIu64 ";\r\n", tail->typeID, tail->pwr_ctr);
	pc.printf("tail.flight_ctr = %" PRIu64 "; \r\ntail.checksum = %08" PRIX32 "\r\n", tail->flight_ctr, tail->checksum);
	pc.printf("tail.magic3 = %08" PRIX32 "\r\n", tail->magic3);
}

void printPacket(void const * packetBytes, uint8_t packetType, Stream & pc)
{
	switch(packetType)
	{
		case LOG_TEXT:
		{
			auto lptxt = reinterpret_cast<struct log_packet_text const *>(packetBytes);
			pc.printf("msg=%s\r\n", lptxt->msg);
        	printPacketTail(&lptxt->tail, pc);
		}
		break;
		case LOG_TEMP:
		{
			auto lptmp = reinterpret_cast<struct log_packet_temp const *>(packetBytes);
			pc.printf("temp=%f\r\n", lptmp->temp);
			printPacketTail(&lptmp->tail, pc);
		}
		break;
		case LOG_ACCEL:
		{
			auto lpaccel = reinterpret_cast<struct log_packet_accel const *>(packetBytes);
			pc.printf("ax=%" PRIi16 "\tay=%" PRIi16 "\taz=%" PRIi16 "\r\n", lpaccel->ax, lpaccel->ay, lpaccel->az);
			printPacketTail(&lpaccel->tail, pc);
		}
		break;
		case LOG_GPS:
		{
			auto lpgps = reinterpret_cast<struct log_packet_gps const *>(packetBytes);
            pc.printf("fixQual=%" PRIu8 "\tnumSats=%d\tlat=%f\tlong=%f\talt=%f"
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
            printPacketTail(&lpgps->tail, pc);
        }
		break;
		case LOG_BNO:
		{
			auto lpbno = reinterpret_cast<struct log_packet_bno const *>(packetBytes);
			pc.printf("qx=%f\tqy=%f\tqz=%f\tqw=%f\tax=%f\tay=%f\taz=%f\tcalib=%hhu\r\n",
        		lpbno->quatX, lpbno->quatY, lpbno->quatZ, lpbno->quatW, lpbno->ax, lpbno->ay, lpbno->az, lpbno->calib);
        	printPacketTail(&lpbno->tail, pc);
		}
		break;
		case LOG_BARO:
		{
			auto lpbaro = reinterpret_cast<struct log_packet_baro const *>(packetBytes);
			pc.printf("altitude=%.02f\tpressure=%" PRIu32 "\ttemp=%.01f\r\n",
        		lpbaro->altitude, lpbaro->pressure, lpbaro->temperature);
			printPacketTail(&lpbaro->tail, pc);
		}
		break;
		case LOG_POWER:
		{
			auto lppower = reinterpret_cast<struct log_packet_power const *>(packetBytes);
			pc.printf("battVoltage=%.02f\tbattCurrent=%.03f\treg5VCurrent=%.02f\tchargePercent=%" PRIu8 "\r\n",
			 	lppower->battVoltage, lppower->battCurrent, lppower->reg5VCurrent, lppower->chargePercent);
			printPacketTail(&lppower->tail, pc);
		}
		break;
		case LOG_ADIS:
		{
			auto lpadis = reinterpret_cast<struct log_packet_adis const *>(packetBytes);
			pc.printf("gyroX=%.02f\tgyroY=%.02f\tgyroZ=%.02f\r\naccelX=%.02f\taccelY=%.02f\taccelZ=%.02f\r\n",
			 	lpadis->gyroX, lpadis->gyroY, lpadis->gyroZ, lpadis->accelX, lpadis->accelY, lpadis->accelZ);
			printPacketTail(&lpadis->tail, pc);
		}
		break;
		case LOG_RANGEFINDER:
		{
			auto lprange = reinterpret_cast<struct log_packet_rangefinder const *>(packetBytes);
			pc.printf("distance=%.02f\trssi=%.01f\tlqi=%" PRIi8 "\r\n",
					  lprange->distance, lprange->rssi, lprange->linkQual);
			printPacketTail(&lprange->tail, pc);
		}
		break;
		default:
			pc.printf("<Unknown packet>\r\n");
	}
}
