/*  USC RPL HAMSTER v2.3 Flash chip and FlashLog tests.
    Lead: Conor Hayes
    Contributors: 
 */

#include <mbed.h>
#include <SerialPort.h>

#include <string>

#include "SPIFBlockDevice.h"

#include "Packet.h"
#include "HamsterFSM.h"
#include "FlashLog.h"

#include "FlashLogTestSuite.h"


/**
 * @brief      Constructs the object.
 */
FlashLogHarness::FlashLogHarness() :
    FlashLog(&powerTimer, &flightTimer, &fsmState)
{
    startTimers();
    fsmState = STATE_STANDBY;

    int err = flashBlockDev.init();
    if (err)
    {
        pc.printf("[SPIFBlockDevice] Error %d initializing device!", err);
    }    


}

/* Call wipeLog(), attempt to use log iterator to read, and confirm 0 packets read. */
int FlashLogHarness::test_chip_erase()
{
    int err;

    pc.printf("Chip initialized. Erasing\r\n");
    err = flashBlockDev.erase(FLASH_START_ADDR, FLASH_END_ADDR-FLASH_START_ADDR);
    if(err)
    {
        pc.printf("erase error %d\r\n", err);
        return err;
    }

    pc.printf("Starting to check chip erase...\r\n");
    check_pattern(0xFFFFFFFF, LOG_START_ADDR, LOG_END_ADDR);

    return 0;
}

/* Write PATTERN across the whole chip, then read the chip back to confirm the writes. */
#define PATTERN 0xdeadbeef
int FlashLogHarness::test_chip_write_pattern()
{
    pc.printf("Starting to write pattern...\r\n");
    return write_pattern(PATTERN, LOG_START_ADDR, LOG_END_ADDR);
}

int FlashLogHarness::test_chip_check_pattern()
{
    pc.printf("Starting to check pattern...\r\n");
    return check_pattern(PATTERN, LOG_START_ADDR, LOG_END_ADDR);
}

/*  Log a repeating pattern of packets using FlashLog until the memory is about half full. 
    After running this test, we'll power cycle and run chip_read_packets OR brownout_recovery. */
#define TP_LIST tp_list1
#define TP_LIST_TYPES tp_list1_types
#define TP_LIST_SIZE TP_LIST1_SIZE
#define AVG_PACKET_SIZE 80
int FlashLogHarness::test_chip_write_packets()
{
    int err = initLog();
    if (err == ERROR_LOG_EXISTS)
    {
        pc.printf("Warning: Log detected.\r\n");
    }
    else if (err == ERROR_SPIFBD_INIT)
    {
        return -2;
    }

    //write a bunch of packets to about a quarter of the log
    pc.printf("Starting to write packets at %08x.\r\n", LOG_START_ADDR);
    int i_max = (LOG_CAPACITY) / (2*AVG_PACKET_SIZE);
    for (int i=0; i<i_max; i++)
    {
        uint8_t type = TP_LIST_TYPES[i % TP_LIST_SIZE];
        void *buf = TP_LIST[i % TP_LIST_SIZE];
        // PRINT_PACKET_BYTES(type, buf);
        writePacket(type, buf);
        if (i % (i_max/100) == 0)
        {
            pc.printf("%.01f%%\r\n", ((double)i/i_max)*100);
        }
    }
    pc.printf("%d packets written to memory. Please power cycle the board!\r\n", i_max);
    return 0;
}

/*  Use the flash block device driver directly to confirm that the repeating pattern of packets
    was written to the flash accurately by chip_write_packets. Print out any mismatches. */  
int FlashLogHarness::test_chip_read_packets()
{

    char buf[MAX_PACKET_LEN];
    int err;
    int i_max = (LOG_END_ADDR - LOG_START_ADDR) / (2*AVG_PACKET_SIZE);
    int type, neq, len, addr = LOG_START_ADDR;

    pc.printf("Beginning to read packets at %08x...\r\n", addr);
    for (int i=0; i<i_max; i++) {       //TODO restore to i_max
        type = TP_LIST_TYPES[i % TP_LIST_SIZE];
        len = getPacketLen(type);
        err = flashBlockDev.read(buf, addr, len);
        if (err)
        {
            return err;
        }
        // compare everything up to the packet tail, which isn't filled in for the test packets.
        neq = strncmp((const char *)buf, (const char *)TP_LIST[i % TP_LIST_SIZE], len - sizeof(struct packet_tail));
        if (neq)
        {
            pc.printf("[EXPECTED]");
            PRETTYPRINT_BYTES(TP_LIST[i % TP_LIST_SIZE], len-sizeof(struct packet_tail), addr);
            pc.printf("[ACTUAL]");
            PRETTYPRINT_BYTES(buf, len, addr);
            pc.printf("\r\n");
        }
        if (i % (i_max/100) == 0)
        {
            pc.printf("--%d%%\r\n", (i*100)/i_max);
        }
        addr += len;
    }
    pc.printf("\n");
    return 0;
}

/* Use the FlashLog's packetIterator function to iterate through the log, checking to see
    that the previous call to chip_write_packets wrote each packet as expected.
    For each packet in the log, prints an error if it's wrong, and nothing if it's right.*/
//#define PRINT_PACKETS
// #define MAX_PACKETS_TO_ITERATE 15
#define PRINT_PACKETS
int FlashLogHarness::test_chip_iterate_packets()
{
    char buf[MAX_PACKET_LEN];
    uint8_t type;
    pc.printf("Iterating through packets in log:\r\n");
    int valid;
    #ifdef MAX_PACKETS_TO_ITERATE
    int iteration_ctr=0;
    #endif
    // As long as our read wasn't out of bounds or from empty/unwritten memory, keep reading (even if there's other types of errors)
    for (valid = packetIterator(&type, buf, true); (valid != ERROR_BOUNDS) && (valid != ERROR_EMPTY); valid = packetIterator(&type, buf, false))
    {
        #ifdef MAX_PACKETS_TO_ITERATE
        if (++iteration_ctr == MAX_PACKETS_TO_ITERATE)
        {
            pc.printf("Ending CHIP_ITERATE_PACKETS -- printed %d packets\r\n", i);
            return 0;
        }
        #endif  //MAX_PACKETS_TO_ITERATE
        switch(valid)
        {
    	case SUCCESS:
            #ifdef PRINT_PACKETS
    		printf("Packet is valid, raw bytes are:\r\n");
    		PRINT_PACKET_BYTES(type, buf);
    		printf("\r\nFormatted data is:\r\n");
            #endif  //PRINT_PACKETS
    		switch(type){
    		case LOG_TEXT:
    			{
    			struct log_packet_text *lptxt = (struct log_packet_text *)buf;
    			#ifdef PRINT_PACKETS
                PRINT_LOG_TEXT(lptxt);
    			#endif //PRINT_PACKETS
                }
    			break;
    		case LOG_TEMP:
    			{
    			struct log_packet_temp *lptmp = (struct log_packet_temp *)buf;
    			#ifdef PRINT_PACKETS
                PRINT_LOG_TEMP(lptmp);
    			#endif //PRINT_PACKETS
                }
    			break;
    		case LOG_ACCEL:
    			{
    			struct log_packet_accel *lpa = (struct log_packet_accel *)buf;
    			#ifdef PRINT_PACKETS
                PRINT_LOG_ACCEL(lpa);
    			#endif //PRINT_PACKETS
                }
    			break;
    		case LOG_BNO:
    			{
    			struct log_packet_bno *lpi = (struct log_packet_bno *)buf;
    			#ifdef PRINT_PACKETS
                PRINT_LOG_BNO(lpi);
    			#endif //PRINT_PACKETS
                }
    			break;
    		case LOG_GPS:
    			{
    			struct log_packet_gps *lpg = (struct log_packet_gps *)buf;
    			#ifdef PRINT_PACKETS
                PRINT_LOG_GPS(lpg);
    			#endif //PRINT_PACKETS
                }
    			break;
    		case LOG_BARO:
    			{
    			struct log_packet_baro *lpb = (struct log_packet_baro *)buf;
    			#ifdef PRINT_PACKETS
                PRINT_LOG_BARO(lpb);
    			#endif //PRINT_PACKETS
                }
    			break;
    		case LOG_POWER:
				{
				struct log_packet_power *lpp = (struct log_packet_power *)buf;
#ifdef PRINT_PACKETS
				PRINT_LOG_POWER(lpp);
#endif //PRINT_PACKETS
				}
            case LOG_ADIS:
                {
                    struct log_packet_adis *lpa = (struct log_packet_adis *)buf;
#ifdef PRINT_PACKETS
                    PRINT_LOG_ADIS(lpa);
#endif //PRINT_PACKETS
                }
    		default:
    			break;
    		}
    		break;
    	case ERROR_CHECKSUM:
    		printf("Error: Invalid checksum or fake magic constant.\r\n");
    		break;
    	case ERROR_TYPE:
    		printf("Error: Invalid packet type.\r\n");
    		break;
    	case ERROR_NOTAIL:
    		printf("Error: No tail/magic constant found in this read.\r\n");
    		break;
    	default:
    		printf("Unrecognized error code: %d\r\n", valid);
    		break;
        }
        if (valid != SUCCESS)
        {
            pc.printf("Buffer contents which caused error:");
            PRETTYPRINT_BYTES(buf, MAX_PACKET_LEN, nextPacketToRead);
        }
    }
    switch(valid){
	case ERROR_BOUNDS:
		printf("Fatal Error: Out of bounds.\r\n\r\n");
		break;
	case ERROR_EMPTY:
		printf("Fatal Error: Completely empty region of memory.\r\n\r\n");
		break;
	default:
		printf("Fatal Error: Unrecognized error code: %d\r\n\r\n", valid);
		break;
    }
    pc.printf("Test complete.\r\n");
    return 0;
}

int FlashLogHarness::test_chip_iterate_gps_text_packets()
{

    char buf[MAX_PACKET_LEN];
    uint8_t type;
    pc.printf("Iterating through GPS and TEXT packets in log:\r\n");
    int valid;
    #ifdef MAX_PACKETS_TO_ITERATE
    int iteration_ctr=0;
    #endif
    // As long as our read wasn't out of bounds or from empty/unwritten memory, keep reading (even if there's other types of errors)
    for (valid = packetIterator(&type, buf, true); (valid != ERROR_BOUNDS) && (valid != ERROR_EMPTY); valid = packetIterator(&type, buf, false))
    {
        #ifdef MAX_PACKETS_TO_ITERATE
        if (++iteration_ctr == MAX_PACKETS_TO_ITERATE)
        {
            pc.printf("Ending CHIP_ITERATE_PACKETS -- printed %d packets\r\n", i);
            return 0;
        }
        #endif  //MAX_PACKETS_TO_ITERATE
        switch(valid)
        {
        case SUCCESS:
            #ifdef PRINT_PACKETS
            pc.printf("Packet is valid, raw bytes are:\r\n");
            PRINT_PACKET_BYTES(type, buf);
            pc.printf("\r\nFormatted data is:\r\n");
            #endif  //PRINT_PACKETS
            switch(type){
            case LOG_TEXT:
                {
                struct log_packet_text *lptxt = (struct log_packet_text *)buf;
                #ifdef PRINT_PACKETS
                PRINT_LOG_TEXT(lptxt);
                #endif //PRINT_PACKETS
                }
                break;
            case LOG_GPS:
                {
                struct log_packet_gps *lpg = (struct log_packet_gps *)buf;
                #ifdef PRINT_PACKETS
                PRINT_LOG_GPS(lpg);
                #endif //PRINT_PACKETS
                }
                break;
            default:
                break;
            }
            break;
        case ERROR_CHECKSUM:
            pc.printf("Error: Invalid checksum or fake magic constant.\r\n");
            break;
        case ERROR_TYPE:
            pc.printf("Error: Invalid packet type.\r\n");
            break;
        case ERROR_NOTAIL:
            pc.printf("Error: No tail/magic constant found in this read.\r\n");
            break;
        default:
            pc.printf("Unrecognized error code: %d\r\n", valid);
            break;
        }
        if (valid != SUCCESS)
        {
            pc.printf("Buffer contents which caused error:");
            PRETTYPRINT_BYTES(buf, MAX_PACKET_LEN, nextPacketToRead);
        }
    }
    switch(valid){
    case ERROR_BOUNDS:
        pc.printf("Fatal Error: Out of bounds.\r\n\r\n");
        break;
    case ERROR_EMPTY:
        pc.printf("Fatal Error: Completely empty region of memory.\r\n\r\n");
        break;
    default:
        pc.printf("Fatal Error: Unrecognized error code: %d\r\n\r\n", valid);
        break;
    }
    pc.printf("Test complete.\r\n");
    return 0;
}






/*  Manually write a bad packet to the end of the flash.
    After running this, confirm that the log can recover by running brownout_recovery 
*/
int FlashLogHarness::test_write_bad_packet()
{

    //initialize the log, which should find the most recent packet.
    int err = initLog();
    if (err == ERROR_LOG_EXISTS)
    {
        pc.printf("restoring pre-existing log\r\n");
        uint64_t restoredPowerTimer;
        uint64_t restoredFlightTimer;
        int err = restoreFSMState(&fsmState, &restoredPowerTimer, &restoredFlightTimer); 
        if (err)
        {
            powerTimer.stop();
            powerTimer.start(restoredPowerTimer);

            flightTimer.stop();
            flightTimer.start(restoredFlightTimer);

            pc.printf("restoreFSMState exited with error %d\r\n", err);
        }
    }
    else if (err == ERROR_SPIFBD_INIT)
    {
        pc.printf("SPIF driver problem. exiting.\r\n");
        return -1;
    }
    else 
    {
        pc.printf("Log empty.\r\n");
    }
    //write a packet that's 1 byte short to simulate a sudden power loss
    flashBlockDev.program(&tp_text, nextPacketAddr, sizeof(tp_text) - 1);
    printLastNBytes(0x100, 0x70);
    pc.printf("Packet partially written. Please power cycle and run brownout_recovery test\r\n");
    return 0;
}

/*  Use FlashLog initLog() to recover from the power cycle by recovering program fsmState. 
    correcting the timer values. */
int FlashLogHarness::test_brownout_recovery()
{

    //initialize the log, which should find the most recent packet.
    int err = initLog();
    if (err == ERROR_LOG_EXISTS) 
    {
        fsmState = STATE_DEBUG;
        pc.printf("PRE-RESTORE:\tflightTimer = %" PRIu64 "ms;\t", flightTimer.read_ms());
        pc.printf("powerTimer = %" PRIu64 "ms;\t", powerTimer.read_ms());
        pc.printf("fsmState = %s\r\n", stringifyState(fsmState));


        uint64_t restoredPowerTimer;
        uint64_t restoredFlightTimer;
        int err = restoreFSMState(&fsmState, &restoredPowerTimer, &restoredFlightTimer); 
        if (err == SUCCESS){

            powerTimer.stop();
            powerTimer.start(restoredPowerTimer);

            flightTimer.stop();
            flightTimer.start(restoredFlightTimer);
        }

        pc.printf("POST-RESTORE:\tflightTimer = %lldms;\t", flightTimer.read_ms());
        pc.printf("powerTimer = %" PRIu64 "ms;\t", powerTimer.read_ms());
        pc.printf("fsmState = %s\r\n", stringifyState(fsmState));
        printLastNBytes(0x50, 0x50);
        pc.printf("restoreFSMState exited with error %d\r\n", err);
        return err;
    }
    else if (err == ERROR_SPIFBD_INIT)
    {
        pc.printf("SPIFBlockDevice driver init error.\r\n");
        return -1;
    }
    else
    {
        pc.printf("BROWNOUT_RECOVERY test FAIL: log not found \r\n");
        return -2;
    }
    
}

/*  Tests that the sequence of chip_write_packets->brownout_recovery has written packets to the memory
    in the manner we expect by reading back all the packets.  */
int FlashLogHarness::test_brownout_recovery_confirm()
{
    return 0;
}

int FlashLogHarness::test_wipe_log()
{
    int err = initLog();
    if (err == ERROR_LOG_EXISTS)
    {
        pc.printf("Wiping an existing log...\r\n");
        wipeLog();
    }
    else if (err == ERROR_SPIFBD_INIT)
    {
        pc.printf("SPIFBlockDevice driver init error.\r\n");
    }
    else
    {
        pc.printf("Log is empty, but wiping it anyway...\r\n");
        wipeLog();
    }

    return err;
}

int FlashLogHarness::test_dump_hex()
{
    pc.printf("Dumping entirety of log into hex...\r\n<hexdump>\r\n");
    struct log_binary_dump_frame frame;
    FLASHLOG_BINARY_ITERATE(this, frame)
    {
        PRINT_BYTES(&frame, sizeof(log_binary_dump_frame));
    }
    pc.printf("</hexdump>\r\n");
    return 0;
}

int FlashLogHarness::test_dump_binary(Serial &pc)
{
    pc.printf("Dumping entirety of log into binary...\r\n<bindump>\r\n");
    struct log_binary_dump_frame frame;
    FLASHLOG_BINARY_ITERATE(this, frame)
    {
       for (size_t i=0; i<sizeof(log_binary_dump_frame); i++)
       {
            while(!pc.writeable()); //wait
            pc.putc(((char*)&frame)[i]);
       } 
    }
    pc.printf("</bindump>\r\n");
    return 0;
}

int FlashLogHarness::test_check_erase()
{
    pc.printf("Confirming erase successful...\r\n");
    int mismatches = check_pattern(0xFFFFFFFF, LOG_START_ADDR, LOG_END_ADDR);
    (mismatches == 0) ? pc.printf("[PASS] ") : pc.printf("[FAIL] ");
    pc.printf("%d errors found.\r\n", mismatches);
    return mismatches;
}
int FlashLogHarness::test_checksum()
{
    /*
    // char *buf = "0123456789012345678901234567890123456789";    //40 characters
    // size_t buf_len = 40;

    void *buf = &tp_text;
    size_t buf_len = sizeof(tp_text);
    buf_len = buf_len-sizeof(struct packet_tail)+offsetof(struct packet_tail, checksum);
    populatePacketTail(LOG_TEXT, sizeof(tp_text), buf);

    pc.printf("Calculating checksum of the following:\r\n");
    PRINT_BYTES(buf, buf_len);
    uint32_t checksum = 0;
    crc32(buf, buf_len, &checksum);
    pc.printf("checksum: %08lu (0x%08x)\r\n", checksum, checksum);
    */
    populatePacketTail(LOG_TEXT, sizeof(tp_text), &tp_text); 
    PRINT_BYTES(&tp_text, sizeof(tp_text)-sizeof(struct packet_tail)+offsetof(struct packet_tail, checksum));
    pc.printf("checksum: %08lu (0x%08lx)\r\n", tp_text.tail.checksum, tp_text.tail.checksum);
    pc.printf("Check the value above against an online crc32 calculator to confirm correct result.\r\n");
    return 1;
}

void FlashLogHarness::startTimers(uint64_t offset)
{
    flightTimer.init();
    powerTimer.init();
    flightTimer.start(offset);
    ThisThread::sleep_for(1s);
    powerTimer.start(offset);
}

int FlashLogHarness::write_pattern(uint32_t pattern, uint32_t start_addr, uint32_t len)
{
    uint32_t buf[0x40];
    //fill the buf with the repeating pattern
    for (int i=0; i<0x40; i++)
    {
        buf[i] = pattern;
    }
    //write the buf to the memory over and over. Writes big chunks to minimize use of the SPI line.
    //rounds down the len to align with the nearest 0x100'th byte
    uint32_t num_steps = len / 0x100;
    uint32_t print_threshold = num_steps / 100;
    uint32_t cur_step = 0;
    for (uint32_t i = start_addr; (i+0x100)<(start_addr+len); i+=0x100)
    {
        //write 256 bytes in a chunk
        int err = flashBlockDev.program(buf, i, 0x100);
        if (err)
        {
            pc.printf("Write error at 0x%08" PRIX32 ".\r\n", i);
            return -1;
        }
        //update a status bar every now and again
        if (++cur_step % print_threshold == 0)
        {
            pc.printf("(%.3f%%)\r\n", ((float)cur_step * 100 / (float)num_steps));
        }
    }
    return 0;
}

int FlashLogHarness::check_pattern(uint32_t pattern, uint32_t start_addr, uint32_t len)
{
    char buf[0x100];
    int mismatch_cnt = 0;
    //rounds down the len to align with the nearest 0x100'th byte
    uint32_t num_steps = len / 0x100;
    uint32_t print_threshold = num_steps / 100;
    uint32_t cur_step = 0;
    for (uint32_t i=start_addr; i+0x100<start_addr+len; i+=0x100)
    {
        int err = flashBlockDev.read(buf, i, 0x100);
        if (err)
        {
            pc.printf("Read error at 0x%08" PRIX32 " .\r\n", i);
            return -1;
        }
        for (int j=0; j<0x40; j++)
        {
            if (((uint32_t *)buf)[j] != pattern)
            {
                pc.printf("[0x%08x] MISMATCH -- expected 0x%08" PRIX32 " -- recieved 0x%08" PRIX32 "\r\n", static_cast<unsigned int>(i+j), pattern, ((uint32_t *)buf)[j]);
                mismatch_cnt++;
            }
        }
        //update a status bar every now and again
        if (++cur_step % print_threshold == 0)
        {
            pc.printf("(%.3f%%)\r\n", ((float)cur_step * 100 / (float)num_steps));
        }
    }
    return mismatch_cnt;
}

int FlashLogHarness::test_writeAtLargeAddress(){

    pc.printf("Chip initialized. \r\n");
    pc.printf("MAX SIZE: %x\r\n", LOG_END_ADDR);
    uint32_t buf = 0xdeadbeef;
    flashBlockDev.program(&buf, 0x03FFFFF0, 4);
    uint32_t frame;
    flashBlockDev.read(&frame, 0x03FFFFF0, 4);
    pc.printf("Read %08" PRIx32 "\r\n", frame);
    return 0;
}

int FlashLogHarness::test_bulkErase(){
    flashBlockDev.bulk_erase();
    test_check_erase();
    return 0;
}


int FlashLogHarness::test_avoid_partial_packet_tail()
{
	// This data string contains a partial packet tail with an invalid state, then
	// a complete INVALID packet.  This reproduced a bug in the pre-Poise version of
	// FlashLog where it would find the partial tail and try to restore from that,
	// using an invalid state and crashing/hanging the code.

    const char badDataString[] = "\xFF\xFF\xFC\xFF\x14\x00\x00\x00\xAA\xAA\xAA\xFF\x03\x00\x00\x00\x30\x38\x1F\x5D\x00\x00\x00\x00\x8E\x41\x54\x3C\x00\x00\x00\x00\xAA\xAA\xAA\xFF\x07\x00\x00\x00\xAA\xAA\xAA\x08\x03\x00\x00\x00\x30\x38\x1F\x5D\x00\x00\x00\x00\x8E\x41\x54\x3C\xCC\xCC\xCC\xCC";

    int err = initLog();
    if (err == ERROR_LOG_EXISTS)
    {
        pc.printf("found pre-existing log\r\n");
    }
    else if (err == ERROR_SPIFBD_INIT)
    {
        pc.printf("SPIF driver problem. exiting.\r\n");
        return -1;
    }
    else 
    {
        pc.printf("Log empty.\r\n");
    }
    pc.printf("Writing bad data starting at %" PRIu32 "\r\n", nextPacketAddr);
    flashBlockDev.program(badDataString, nextPacketAddr, sizeof(badDataString) - 1);
    pc.printf("Bad packet written. Please power cycle and run brownout_recovery test\r\n");
    return 0;
}


/* Add a case to the switch statement, and an option to the menu, when adding a new test case function above. */
#if HAMSTER_SIMULATOR != 1
int main()
#else
int flash_test_main()
#endif
{
    //declare the test harness
    FlashLogHarness harness;
    harness.initLog();

    while(1)
    {
        int test=-1;
        pc.printf("\r\n\nHamster Flash Test Suite:\r\n");

        //MENU. ADD AN OPTION FOR EACH TEST.
        pc.printf("Select a test: \n\r");
        pc.printf("1.  Exit test suite\r\n");
        pc.printf("2.  Chip Erase\r\n");
        pc.printf("3.  Chip Write Pattern\r\n");
        pc.printf("4.  Chip Check Pattern\r\n");
        pc.printf("5.  Chip Write Packets\r\n");
        pc.printf("6.  Chip Check Packets\r\n");
        pc.printf("7.  Chip Iterate Packets\r\n");
        pc.printf("8.  Write Bad Packet\r\n");
        pc.printf("9.  Brownout Recovery\r\n");
        pc.printf("10. Confirm Brownout Recovery\r\n");
        pc.printf("11. Wipe Log\r\n");
        pc.printf("12. Dump Hex Data\r\n");
        pc.printf("13. Dump Binary Data\r\n");
        pc.printf("14. GPS Text Packet iterate\r\n");
        pc.printf("15. Test checksum\r\n");
        pc.printf("16. Check Erase\r\n");
        pc.printf("17. Write at Large Address\r\n");
        pc.printf("18. Bulk Erase\r\n");
        pc.printf("19. Write Magic Bad Packet\r\n");
        
        pc.scanf("%d", &test);
        pc.printf("Running test %d:\r\n\n", test);

        //SWITCH. ADD A CASE FOR EACH TEST.
        switch(test)
        {
            case 1:         pc.printf("Exiting test suite.\r\n");          return 0;
            case 2:         harness.test_chip_erase();                    break;
            case 3:         harness.test_chip_write_pattern();            break;
            case 4:         harness.test_chip_check_pattern();            break;
            case 5:         harness.test_chip_write_packets();            break;
            case 6:         harness.test_chip_read_packets();             break;
            case 7:         harness.test_chip_iterate_packets();          break;
            case 8:         harness.test_write_bad_packet();              break;
            case 9:         harness.test_brownout_recovery();             break;
            case 10:        harness.test_brownout_recovery_confirm();     break;
            case 11:        harness.test_wipe_log();                      break;
            case 12:        harness.test_dump_hex();                      break;
            case 13:        harness.test_dump_binary(pc);                 break;
            case 14:        harness.test_chip_iterate_gps_text_packets(); break;
            case 15:        harness.test_checksum();                      break;
            case 16:        harness.test_check_erase();                   break;
            case 17:        harness.test_writeAtLargeAddress();           break;
            case 18:        harness.test_bulkErase();                     break;
            case 19:
				harness.test_avoid_partial_packet_tail();       break;
            default:        pc.printf("Invalid test number. Please run again.\r\n"); return 1;
        }

        pc.printf("done.\r\n");
    }
}

