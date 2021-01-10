#pragma clang diagnostic push
#pragma ide diagnostic ignored "readability-magic-numbers"
/*  USC RPL HAMSTER v2.3 Flash chip and FlashLog tests.
    Lead: Conor Hayes
    Contributors: 
 */

#include <mbed.h>
#include <SerialStream.h>

#include <string>


#include "Packet.h"
#include "FlashLog.h"

#include "FlashLogTestSuite.h"

BufferedSerial serial(USBTX, USBRX, 115200);
SerialStream<BufferedSerial> pcStream(serial); // can't name it pc due to conflict with FlashLog class variable

BLOCK_DEVICE_CONSTRUCTOR

/**
 * @brief      Constructs the object.
 */
FlashLogHarness::FlashLogHarness() :
FlashLog(harnessBlockDev, pcStream)
{
    startTimers();

    // init block device now so
    int err = harnessBlockDev.init();
    if(err != BD_ERROR_OK)
	{
    	pc.printf("Failed to init block device, reports error %d\r\n", err);
	}
}

/* Call wipeLog(), attempt to use log iterator to read, and confirm 0 packets read. */
int FlashLogHarness::test_chip_erase()
{
    int err;

    pc.printf("Chip initialized. Erasing\r\n");
    err = sdBlockDev.erase(logStart, logEnd);
    if(err)
    {
        pc.printf("erase error %d\r\n", err);
        return err;
    }

    pc.printf("Starting to check chip erase...\r\n");
    check_pattern(0xFFFFFFFF, logStart, logEnd);

    return 0;
}

/* Write PATTERN across the whole chip, then read the chip back to confirm the writes. */
#define PATTERN 0xdeadbeef
int FlashLogHarness::test_chip_write_pattern()
{
    pc.printf("Starting to write pattern...\r\n");
    return write_pattern(PATTERN, logStart, logEnd);
}

int FlashLogHarness::test_chip_check_pattern()
{
    pc.printf("Starting to check pattern...\r\n");
    return check_pattern(PATTERN, logStart, logEnd);
}

/*  Log a repeating pattern of packets using FlashLog until the memory is about half full. 
    After running this test, we'll power cycle and run chip_read_packets OR brownout_recovery. */
#define TP_LIST tp_list1
#define TP_LIST_TYPES tp_list1_types
#define TP_LIST_SIZE TP_LIST1_SIZE
#define AVG_PACKET_SIZE 80
int FlashLogHarness::test_chip_write_packets()
{
	std::pair<bd_error, FLResultCode> err = initLog();
    if (err.second == FL_ERROR_LOG_EXISTS)
    {
        pc.printf("Warning: Log detected.\r\n");
    }
    else if (err.second == FL_ERROR_BD_INIT)
    {
        return -2;
    }

    //write a bunch of packets to about a quarter of the log
    pc.printf("Starting to write packets at %08" PRIu64 ".\r\n", logStart);
    int i_max = (getLogCapacity()) / (2*AVG_PACKET_SIZE);
    for (int i=0; i<i_max; i++)
    {
        uint8_t type = TP_LIST_TYPES[i % TP_LIST_SIZE];
        void *buf = TP_LIST[i % TP_LIST_SIZE];
        // PRINT_PACKET_BYTES(type, buf);
        writePacket(type, buf,
					std::chrono::duration_cast<std::chrono::microseconds>(powerTimer.elapsed_time()).count(),
					std::chrono::duration_cast<std::chrono::microseconds>(flightTimer.elapsed_time()).count(), fsmState);
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
    bd_size_t i_max = (logEnd - logStart) / (2*AVG_PACKET_SIZE);
    int type, neq, len, addr = logStart;

    pc.printf("Beginning to read packets at %08x...\r\n", addr);
    for (bd_size_t i=0; i<i_max; i++) {       //TODO restore to i_max
        type = TP_LIST_TYPES[i % TP_LIST_SIZE];
        len = getPacketLen(type);
        err = sdBlockDev.read(buf, addr, len);
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
            pc.printf("--%" PRIu64 "%%\r\n", (i*100)/i_max);
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
    for (valid = packetIterator(&type, buf, true); (valid != FL_ERROR_BOUNDS) && (valid != FL_ERROR_EMPTY); valid = packetIterator(&type, buf, false))
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
    	case FL_SUCCESS:
            #ifdef PRINT_PACKETS
    		printf("Packet is valid, raw bytes are:\r\n");
    		PRINT_PACKET_BYTES(type, buf);
    		printf("\r\nFormatted data is:\r\n");
            #endif  //PRINT_PACKETS

    		printPacket(buf, type, pc);
    		break;
    	case FL_ERROR_CHECKSUM:
    		printf("Error: Invalid checksum or fake magic constant.\r\n");
    		break;
    	case FL_ERROR_TYPE:
    		printf("Error: Invalid packet type.\r\n");
    		break;
    	case FL_ERROR_NOTAIL:
    		printf("Error: No tail/magic constant found in this read.\r\n");
    		break;
    	default:
    		printf("Unrecognized error code: %d\r\n", valid);
    		break;
        }
        if (valid != FL_SUCCESS)
        {
            pc.printf("Buffer contents which caused error:");
            PRETTYPRINT_BYTES(buf, MAX_PACKET_LEN, nextPacketToRead);
        }
    }
    switch(valid){
	case FL_ERROR_BOUNDS:
		printf("Fatal Error: Out of bounds.\r\n\r\n");
		break;
	case FL_ERROR_EMPTY:
		printf("Fatal Error: Completely empty region of memory.\r\n\r\n");
		break;
	default:
		printf("Fatal Error: Unrecognized error code: %d\r\n\r\n", valid);
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
	std::pair<bd_error, FLResultCode> err = initLog();
    if (err.second == FL_ERROR_LOG_EXISTS)
    {
        pc.printf("restoring pre-existing log\r\n");
        uint64_t restoredPowerTimer;
        uint64_t restoredFlightTimer;
        FLResultCode restoreErr = restoreFSMState(&fsmState, &restoredPowerTimer, &restoredFlightTimer);
        if (restoreErr != FL_SUCCESS)
        {
            pc.printf("restoreFSMState exited with error %d\r\n", restoreErr);
        }
        else
		{
			pc.printf("Log restored state as powerTimer=%" PRIu64 ", flightTimer=%" PRIu64 ", state=%s\r\n", restoredPowerTimer, restoredFlightTimer, FL_GET_STATE_NAME(fsmState));
		}
    }
    else if (err.second == FL_ERROR_BD_INIT)
    {
        pc.printf("SPIF driver problem. exiting.\r\n");
        return -1;
    }
    else 
    {
        pc.printf("Log empty.\r\n");
    }
    //write a packet that's 1 byte short to simulate a sudden power loss
    sdBlockDev.program(&tp_text, nextPacketAddr, sizeof(tp_text) - 1);
    printLastNBytes(0x100, 0x70);
    pc.printf("Packet partially written. Please power cycle and run brownout_recovery test\r\n");
    return 0;
}

/*  Use FlashLog initLog() to recover from the power cycle by recovering program fsmState. 
    correcting the timer values. */
int FlashLogHarness::test_brownout_recovery()
{

    //initialize the log, which should find the most recent packet.
	std::pair<bd_error, FLResultCode> err = initLog();
    if (err.second == FL_ERROR_LOG_EXISTS)
    {
        fsmState = FL_HARNESS_EXAMPLE_STATE;
        pc.printf("PRE-RESTORE:\tflightTimer = %" PRIu64 "ms;\t", flightTimer.read_ms());
        pc.printf("powerTimer = %" PRIu64 "ms;\t", powerTimer.read_ms());
        pc.printf("fsmState = %s\r\n", FL_GET_STATE_NAME(fsmState));


        uint64_t restoredPowerTimer;
        uint64_t restoredFlightTimer;
        int err = restoreFSMState(&fsmState, &restoredPowerTimer, &restoredFlightTimer); 
        if (err == FL_SUCCESS){

        	// TODO implement restoring timers
            //powerTimer.stop();
            //powerTimer.start(restoredPowerTimer);

            //flightTimer.stop();
            //flightTimer.start(restoredFlightTimer);
        }

        pc.printf("POST-RESTORE:\tflightTimer = %lldms;\t", flightTimer.read_ms());
        pc.printf("powerTimer = %" PRIu64 "ms;\t", powerTimer.read_ms());
        pc.printf("fsmState = %s\r\n", FL_GET_STATE_NAME(fsmState));
        printLastNBytes(0x50, 0x50);
        pc.printf("restoreFSMState exited with error %d\r\n", err);
        return err;
    }
    else if (err.second == FL_ERROR_BD_INIT)
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
	std::pair<bd_error, FLResultCode> err = initLog();
    if (err.second == FL_ERROR_LOG_EXISTS)
    {
        pc.printf("Wiping an existing log...\r\n");
        wipeLog();
    }
    else if (err.second == FL_ERROR_BD_INIT)
    {
        pc.printf("SPIFBlockDevice driver init error.\r\n");
    }
    else
    {
        pc.printf("Log is empty, but wiping it anyway...\r\n");
        wipeLog();
    }

    return err.second;
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

int FlashLogHarness::test_dump_binary(Stream &pc)
{
    pc.printf("Dumping entirety of log into binary...\r\n<bindump>\r\n");
    struct log_binary_dump_frame frame;
    FLASHLOG_BINARY_ITERATE(this, frame)
    {
       for (size_t i=0; i<sizeof(log_binary_dump_frame); i++)
       {
            pc.putc(((char*)&frame)[i]);
       } 
    }
    pc.printf("</bindump>\r\n");
    return 0;
}

int FlashLogHarness::test_check_erase()
{
    pc.printf("Confirming erase FL_SUCCESSful...\r\n");
    int mismatches = check_pattern(0xFFFFFFFF, logStart, logEnd);
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
    populatePacketTail(LOG_TEXT, sizeof(tp_text), &tp_text, 0, 0, FL_HARNESS_EXAMPLE_STATE);
    PRINT_BYTES(&tp_text, sizeof(tp_text)-sizeof(struct packet_tail)+offsetof(struct packet_tail, checksum));
    pc.printf("checksum: %08lu (0x%08lx)\r\n", tp_text.tail.checksum, tp_text.tail.checksum);
    pc.printf("Check the value above against an online crc32 calculator to confirm correct result.\r\n");
    return 1;
}

void FlashLogHarness::startTimers(uint64_t offset)
{
	// TODO implement timer offsets
    /*flightTimer.init();
    powerTimer.init();
    flightTimer.start(offset);
    ThisThread::sleep_for(1s);
    powerTimer.start(offset);*/
}

int FlashLogHarness::write_pattern(uint32_t pattern, bd_addr_t start_addr, bd_size_t len)
{
    static uint32_t buf[FL_MAX_BLOCK_SIZE / 4]; // divide byte block size by 4 because buffer is 32 bit ints
    //fill the buf with the repeating pattern
    for (int i=0; i<blockSize / 4; i++)
    {
        buf[i] = pattern;
    }

    //write the buf to the memory over and over. Writes big chunks to minimize use of the SPI line.
    //rounds down the len to align with the nearest 0x100'th byte
    bd_size_t num_steps = len / blockSize;
    uint32_t print_threshold = num_steps / 100;
    bd_addr_t cur_step = 0;
    for (bd_addr_t i = start_addr; (i+blockSize)<(start_addr+len); i+=blockSize)
    {
        //write 256 bytes in a chunk
        int err = sdBlockDev.program(buf, i, blockSize);
        if (err)
        {
            pc.printf("Write error at 0x%08" PRIX64 ".\r\n", i);
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

int FlashLogHarness::check_pattern(uint32_t pattern, bd_addr_t start_addr, bd_size_t len)
{
    static char buf[FL_MAX_BLOCK_SIZE];
    int mismatch_cnt = 0;
    //rounds down the len to align with the nearest 0x100'th byte
    uint32_t num_steps = len / blockSize;
    uint32_t print_threshold = num_steps / 100;
    uint32_t cur_step = 0;
    for (bd_addr_t i=start_addr; i+blockSize<start_addr+len; i+=blockSize)
    {
        int err = sdBlockDev.read(buf, i, blockSize);
        if (err)
        {
            pc.printf("Read error at 0x%08" PRIX64 " .\r\n", i);
            return -1;
        }
        // check 4 bytes at a time
        for (int j=0; j<blockSize / 4; j++)
        {
            if(reinterpret_cast<uint32_t *>(buf)[j] != pattern)
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
    pc.printf("MAX SIZE: %" PRIx64 "\r\n", logEnd);
    uint32_t buf = 0xdeadbeef;
    sdBlockDev.program(&buf, 0x03FFFFF0, 4);
    uint32_t frame;
    sdBlockDev.read(&frame, 0x03FFFFF0, 4);
    pc.printf("Read %08" PRIx32 "\r\n", frame);
    return 0;
}

int FlashLogHarness::test_bulkErase(){

#if FL_IS_SPI_FLASH
	sdBlockDev.bulk_erase();
    test_check_erase();
#else
    pc.printf("Bulk erase not implemented!\r\n");
#endif
    return 0;
}


int FlashLogHarness::test_avoid_partial_packet_tail()
{
	// This data string contains a partial packet tail with an invalid state, then
	// a complete INVALID packet.  This reproduced a bug in the pre-Poise version of
	// FlashLog where it would find the partial tail and try to restore from that,
	// using an invalid state and crashing/hanging the code.

    const char badDataString[] = "\xFF\xFF\xFC\xFF\x14\x00\x00\x00\xAA\xAA\xAA\xFF\x03\x00\x00\x00\x30\x38\x1F\x5D\x00\x00\x00\x00\x8E\x41\x54\x3C\x00\x00\x00\x00\xAA\xAA\xAA\xFF\x07\x00\x00\x00\xAA\xAA\xAA\x08\x03\x00\x00\x00\x30\x38\x1F\x5D\x00\x00\x00\x00\x8E\x41\x54\x3C\xCC\xCC\xCC\xCC";

	std::pair<bd_error, FLResultCode> err = initLog();
    if (err.second == FL_ERROR_LOG_EXISTS)
    {
        pc.printf("found pre-existing log\r\n");
    }
    else if (err.second == FL_ERROR_BD_INIT)
    {
        pc.printf("SPIF driver problem. exiting.\r\n");
        return -1;
    }
    else 
    {
        pc.printf("Log empty.\r\n");
    }
    pc.printf("Writing bad data starting at 0x%" PRIx64 "\r\n", nextPacketAddr);
    sdBlockDev.program(badDataString, nextPacketAddr, sizeof(badDataString) - 1);
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
        pcStream.printf("\r\n\nHamster Flash Test Suite:\r\n");

        //MENU. ADD AN OPTION FOR EACH TEST.
        pcStream.printf("Select a test: \n\r");
        pcStream.printf("1.  Exit test suite\r\n");
        pcStream.printf("2.  Chip Erase\r\n");
        pcStream.printf("3.  Chip Write Pattern\r\n");
        pcStream.printf("4.  Chip Check Pattern\r\n");
        pcStream.printf("5.  Chip Write Packets\r\n");
        pcStream.printf("6.  Chip Check Packets\r\n");
        pcStream.printf("7.  Chip Iterate Packets\r\n");
        pcStream.printf("8.  Write Bad Packet\r\n");
        pcStream.printf("9.  Brownout Recovery\r\n");
        pcStream.printf("10. Confirm Brownout Recovery\r\n");
        pcStream.printf("11. Wipe Log\r\n");
        pcStream.printf("12. Dump Hex Data\r\n");
        pcStream.printf("13. Dump Binary Data\r\n");
        pcStream.printf("15. Test checksum\r\n");
        pcStream.printf("16. Check Erase\r\n");
        pcStream.printf("17. Write at Large Address\r\n");
        pcStream.printf("18. Bulk Erase\r\n");
        pcStream.printf("19. Write Magic Bad Packet\r\n");

        pcStream.scanf("%d", &test);
        pcStream.printf("Running test %d:\r\n\n", test);

        //SWITCH. ADD A CASE FOR EACH TEST.
        switch(test)
        {
            case 1:         pcStream.printf("Exiting test suite.\r\n");          return 0;
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
            case 13:        harness.test_dump_binary(pcStream);                 break;
            case 15:        harness.test_checksum();                      break;
            case 16:        harness.test_check_erase();                   break;
            case 17:        harness.test_writeAtLargeAddress();           break;
            case 18:        harness.test_bulkErase();                     break;
            case 19:
				harness.test_avoid_partial_packet_tail();       break;
            default:        pcStream.printf("Invalid test number. Please run again.\r\n"); return 1;
        }

        pcStream.printf("done.\r\n");
    }
}


#pragma clang diagnostic pop