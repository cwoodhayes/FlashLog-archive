#pragma clang diagnostic push
#pragma ide diagnostic ignored "readability-magic-numbers"
/*  USC RPL HAMSTER v2.3 Flash chip and FlashLog tests.
    Lead: Conor Hayes
    Contributors: 
 */

#include "FlashLogTestSuite.h"

#include <mbed.h>

#include <string>

std::unique_ptr<FlashLogConfig::BlockDevice_t> harnessBlockDev = FlashLogConfig::constructBlockDevice();
std::unique_ptr<FlashLogConfig::FileHandle_t> serialPort = FlashLogConfig::constructFileHandle();

/**
 * @brief Override (retarget) the console with file handle specified in FlashLogConfig
 *
 * This lets calls like `printf` and `scanf` work with the correct output device.
 */
FileHandle* mbed::mbed_override_console(int fd)
{
    return serialPort.get();
}

/**
 * @brief      Constructs the object.
 */
FlashLogHarness::FlashLogHarness() :
FlashLog(*harnessBlockDev)
{
    startTimers();

    // init block device now so
    int err = harnessBlockDev->init();
    if(err != BD_ERROR_OK)
	{
    	printf("Failed to init block device, reports error %d\r\n", err);
	}
}

/* Attempt to erase the whole chip (up until #logEnd) in one BlockDevice::erase call */
int FlashLogHarness::test_chip_erase()
{
    int err;

    printf("Chip initialized. Erasing\r\n");
    /* The address we want to erase to must be a multiple of the erase size of the device. We want
     * to get the next address that is a multiple of the erase size, so we "round up" (but note that
     * if our address is exactly a multiple of the erase size, we should not increase our erase
     * address). Use integer division rounding to our advantage here:
     * (int)N / (int)D = floor((float)N / (float)D)), so (aN+x)/N = a (x < N), and
     * ((a-1)N + x-1+N)/N = a, (x <= N) */
    const bd_addr_t eraseSize
        = (((logEnd - 1 + harnessBlockDev->get_erase_size()) / harnessBlockDev->get_erase_size())
              * harnessBlockDev->get_erase_size())
        - logStart;
    err = harnessBlockDev->erase(logStart, eraseSize);
    if(err)
    {
        printf("erase error %d\r\n", err);
        return err;
    }

    printf("Starting to check chip erase...\r\n");
    check_pattern(0xFFFFFFFF, logStart, logEnd);

    return 0;
}

/* Write PATTERN across the whole chip, then read the chip back to confirm the writes. */
#define PATTERN 0xdeadbeef
int FlashLogHarness::test_chip_write_pattern()
{
    printf("Starting to write pattern...\r\n");
    return write_pattern(PATTERN, logStart, logEnd);
}

int FlashLogHarness::test_chip_write_pattern_through_log()
{
	printf("Starting to write pattern...\r\n");
	return write_pattern_through_flashlog(PATTERN, logStart, logEnd);
}


int FlashLogHarness::test_chip_check_pattern()
{
    printf("Starting to check pattern...\r\n");
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
        printf("Warning: Log detected.\r\n");
    }
    else if (err.second == FL_ERROR_BD_INIT)
    {
        return -2;
    }

    //write a bunch of packets to about a quarter of the log
    printf("Starting to write packets at %08" PRIu64 ".\r\n", logStart);
    int i_max = (getLogCapacity()) / (2*AVG_PACKET_SIZE);
    for (int i=0; i<i_max; i++)
    {
        uint8_t type = TP_LIST_TYPES[i % TP_LIST_SIZE];
        void *buf = TP_LIST[i % TP_LIST_SIZE];
        // PRINT_PACKET_BYTES(type, buf);
        writePacket(type, buf, getPowerTimer(), getFlightTimer(), fsmState);
        if (i % (i_max/100) == 0)
        {
            printf("%.01f%%\r\n", ((double)i/i_max)*100);
        }
    }
    printf("%d packets written to memory. Please power cycle the board!\r\n", i_max);
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

    printf("Beginning to read packets at %08x...\r\n", addr);
    for (bd_size_t i=0; i<i_max; i++) {       //TODO restore to i_max
        type = TP_LIST_TYPES[i % TP_LIST_SIZE];
        len = getPacketLen(type);
        err = harnessBlockDev->read(buf, addr, len);
        if (err)
        {
            return err;
        }
        // compare everything up to the packet tail, which isn't filled in for the test packets.
        neq = strncmp((const char *)buf, (const char *)TP_LIST[i % TP_LIST_SIZE], len - sizeof(struct packet_tail));
        if (neq)
        {
            printf("[EXPECTED]");
            PRETTYPRINT_BYTES(TP_LIST[i % TP_LIST_SIZE], len-sizeof(struct packet_tail), addr);
            printf("[ACTUAL]");
            PRETTYPRINT_BYTES(buf, len, addr);
            printf("\r\n");
        }
        if (i % (i_max/100) == 0)
        {
            printf("--%" PRIu64 "%%\r\n", (i*100)/i_max);
        }
        addr += len;
    }
    printf("\n");
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
    printf("Iterating through packets in log:\r\n");
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
            printf("Ending CHIP_ITERATE_PACKETS -- printed %d packets\r\n", i);
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

    		printPacket(buf, type);
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
            printf("Buffer contents which caused error:");
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
    printf("Test complete.\r\n");
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
        printf("restoring pre-existing log\r\n");
        uint64_t restoredPowerTimer;
        uint64_t restoredFlightTimer;
        FLResultCode restoreErr = restoreFSMState(&fsmState, &restoredPowerTimer, &restoredFlightTimer);
        if (restoreErr != FL_SUCCESS)
        {
            printf("restoreFSMState exited with error %d\r\n", restoreErr);
        }
        else
		{
			printf("Log restored state as powerTimer=%" PRIu64 ", flightTimer=%" PRIu64 ", state=%s\r\n", restoredPowerTimer, restoredFlightTimer, FlashLogConfig::getStateName(fsmState));
		}
    }
    else if (err.second == FL_ERROR_BD_INIT)
    {
        printf("SPIF driver problem. exiting.\r\n");
        return -1;
    }
    else 
    {
        printf("Log empty.\r\n");
    }
    //write a packet that's 1 byte short to simulate a sudden power loss
    harnessBlockDev->program(&tp_text, nextPacketAddr, sizeof(tp_text) - 1);
    printLastNBytes(0x100, 0x70);
    printf("Packet partially written. Please power cycle and run brownout_recovery test\r\n");
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
        fsmState = FlashLogConfig::harnessExampleState;
        printf("PRE-RESTORE:\tflightTimer = %" PRIu64 "us;\t", getFlightTimer());
        printf("powerTimer = %" PRIu64 "us;\t", getPowerTimer());
        printf("fsmState = %s\r\n", FlashLogConfig::getStateName(fsmState));


        uint64_t restoredPowerTimer;
        uint64_t restoredFlightTimer;
        int err = restoreFSMState(&fsmState, &restoredPowerTimer, &restoredFlightTimer); 
        if (err == FL_SUCCESS)
        {

        	powerTimer.stop();
        	powerTimer.reset();
        	powerTimerOffset = restoredPowerTimer;
        	powerTimer.start();

			flightTimer.stop();
			flightTimer.reset();
			flightTimerOffset = restoredFlightTimer;
			flightTimer.start();
        }

		printf("POST-RESTORE:\tflightTimer = %" PRIu64 "us;\t", getFlightTimer());
		printf("powerTimer = %" PRIu64 "us;\t", getPowerTimer());
        printf("fsmState = %s\r\n", FlashLogConfig::getStateName(fsmState));
        printLastNBytes(0x50, 0x50);
        printf("restoreFSMState exited with error %d\r\n", err);
        return err;
    }
    else if (err.second == FL_ERROR_BD_INIT)
    {
        printf("SPIFBlockDevice driver init error.\r\n");
        return -1;
    }
    else
    {
        printf("BROWNOUT_RECOVERY test FAIL: log not found \r\n");
        return -2;
    }
    
}

/*  Tests that the sequence of chip_write_packets->brownout_recovery has written packets to the memory
    in the manner we expect by reading back all the packets.  */
int FlashLogHarness::test_brownout_recovery_confirm()
{
    return 0;
}

int FlashLogHarness::test_wipe_log(bool completeErase)
{
	std::pair<bd_error, FLResultCode> err = initLog();
    if (err.second == FL_ERROR_LOG_EXISTS)
    {
        printf("Wiping an existing log...\r\n");
    }
    else if (err.second == FL_ERROR_BD_INIT)
    {
        printf("SPIFBlockDevice driver init error.\r\n");
		return err.second;
	}
    else
    {
        printf("Log is empty, but wiping it anyway...\r\n");
    }

	float prevProgress = 0;
	wipeLog(completeErase, [&](float progress)
	{
		if(progress - prevProgress > 0.1f)
		{
			printf("(%.02f%%)\r\n", progress);
			prevProgress = progress;
		}
	});


	return err.second;
}

int FlashLogHarness::test_dump_hex()
{
    printf("Dumping entirety of log into hex...\r\n<hexdump>\r\n");
    static struct log_binary_dump_frame frame;
    FLASHLOG_BINARY_ITERATE(this, frame)
    {
        PRINT_BYTES(&frame, sizeof(log_binary_dump_frame));
    }
    printf("</hexdump>\r\n");
    return 0;
}

struct log_binary_dump_frame frame;
int FlashLogHarness::test_dump_binary()
{
    fflush(stdout);
    ThisThread::sleep_for(100ms);
    serialPort->set_baud(921600);

	// Give the PC time to switch its baudrate
	ThisThread::sleep_for(100ms);

    printf("Dumping entirety of log into binary...\r\n<bindump>\r\n");
    FLASHLOG_BINARY_ITERATE(this, frame)
    {
       for (size_t i=0; i<sizeof(log_binary_dump_frame); i++)
       {
            printf("%c", ((char*)&frame)[i]);
       }
    }
    printf("</bindump>\r\n");

    fflush(stdout);
    ThisThread::sleep_for(100ms);
    serialPort->set_baud(115200);

	return 0;
}

int FlashLogHarness::test_check_erase()
{
    printf("Confirming erase FL_SUCCESSful...\r\n");
    int mismatches = check_pattern(0xFFFFFFFF, logStart, logEnd);
    (mismatches == 0) ? printf("[PASS] ") : printf("[FAIL] ");
    printf("%d errors found.\r\n", mismatches);
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

    printf("Calculating checksum of the following:\r\n");
    PRINT_BYTES(buf, buf_len);
    uint32_t checksum = 0;
    crc32(buf, buf_len, &checksum);
    printf("checksum: %08lu (0x%08x)\r\n", checksum, checksum);
    */
    populatePacketTail(LOG_TEXT, sizeof(tp_text), &tp_text, 0, 0, FlashLogConfig::harnessExampleState);
    PRINT_BYTES(&tp_text, sizeof(tp_text)-sizeof(struct packet_tail)+offsetof(struct packet_tail, checksum));
    printf("checksum: %08lu (0x%08lx)\r\n", tp_text.tail.checksum, tp_text.tail.checksum);
    printf("Check the value above against an online crc32 calculator to confirm correct result.\r\n");
    return 1;
}

void FlashLogHarness::startTimers(uint64_t offset)
{
    flightTimer.reset();
    powerTimer.reset();

    // set flight timer 1 sec ahead of flight timer
    flightTimerOffset = offset;
    powerTimerOffset = offset + 1e6;
}

template <typename T, size_t SIZE>
static inline constexpr size_t arr_sizeof(const T (&arr)[SIZE]) noexcept
{
    return SIZE;
}

int FlashLogHarness::write_pattern(uint32_t pattern, bd_addr_t start_addr, bd_size_t len)
{
    static uint32_t buf[FlashLogConfig::maxBlockSize / 4]; // divide byte block size by 4 because buffer is 32 bit ints
    static constexpr size_t buf_size = arr_sizeof(buf);

    //fill the buf with the repeating pattern
    for (bd_size_t i=0; i<buf_size / 4; i++)
    {
        buf[i] = pattern;
    }

    //write the buf to the memory over and over. Writes big chunks to minimize use of the SPI line.
    //rounds down the len to align with the nearest 0x100'th byte
    bd_size_t num_steps = len / buf_size;
    uint32_t print_threshold = num_steps / 100;
    bd_addr_t cur_step = 0;
    for (bd_addr_t i = start_addr; (i+buf_size)<(start_addr+len); i+=buf_size)
    {
        //write 256 bytes in a chunk
        int err = harnessBlockDev->program(buf, i, buf_size);
        if (err)
        {
            printf("Write error at 0x%08" PRIX64 ".\r\n", i);
            return -1;
        }
        //update a status bar every now and again
        if (++cur_step % print_threshold == 0)
        {
            printf("(%.3f%%)\r\n", ((float)cur_step * 100 / (float)num_steps));
        }
    }
    return 0;
}

int FlashLogHarness::write_pattern_through_flashlog(uint32_t pattern, const bd_addr_t start_addr, const bd_size_t len)
{
	static uint32_t buf[(2*FlashLogConfig::maxBlockSize) / 4]; // divide byte block size by 4 because buffer is 32 bit ints
    static constexpr size_t buf_size = arr_sizeof(buf);

	//fill the buf with the repeating pattern
	for (bd_size_t i=0; i<buf_size / 4; i++)
	{
		buf[i] = pattern;
	}

	//write the buf to the memory over and over. Writes big chunks to minimize use of the SPI line.
	//rounds down the len to align with the nearest 0x100'th byte
	bd_size_t num_steps = len / blockSize;
	uint32_t print_threshold = num_steps / 100;
	bd_addr_t cur_step = 0;

	// current size of the individual block being programmed.
	// Changes over the course of the test.
	bd_size_t programBlockSize = sizeof(uint32_t);

	for (bd_addr_t i = start_addr; i < start_addr + len;)
	{
		// reduce the size of this program if we are near the end of the programming operation
		bd_addr_t nextStartAddr = i + programBlockSize;
		if(nextStartAddr > start_addr + len)
		{
			nextStartAddr = start_addr + len;
		}

		//write 256 bytes in a chunk
		int err = writeToLog(buf, i, (nextStartAddr - i));
		if (err)
		{
			printf("Write error at 0x%08" PRIX64 ".\r\n", i);
			return -1;
		}

		i = nextStartAddr;

		//update a status bar every now and again
		if (++cur_step % print_threshold == 0)
		{
			printf("(%.3f%%)\r\n", static_cast<float>(i - start_addr)/static_cast<float>(len)*100);
		}

		// change block size (increment or reset)
		programBlockSize = (programBlockSize % buf_size) + sizeof(uint32_t);
	}
	return 0;
}

int FlashLogHarness::check_pattern(uint32_t pattern, bd_addr_t start_addr, bd_size_t len)
{
    static char buf[FlashLogConfig::maxBlockSize];
    static constexpr size_t buf_size = arr_sizeof(buf);

    int mismatch_cnt = 0;
    //rounds down the len to align with the nearest 0x100'th byte
    uint32_t num_steps = len / buf_size;
    uint32_t print_threshold = num_steps / 100;
    uint32_t cur_step = 0;
    for (bd_addr_t i=start_addr; i+buf_size<start_addr+len; i+=buf_size)
    {
        int err = harnessBlockDev->read(buf, i, buf_size);
        if (err)
        {
            printf("Read error at 0x%08" PRIX64 " .\r\n", i);
            return -1;
        }
        // check 4 bytes at a time
        for (bd_size_t j=0; j<buf_size / 4; j++)
        {
            if(reinterpret_cast<uint32_t *>(buf)[j] != pattern)
            {
                printf("[0x%08x] MISMATCH -- expected 0x%08" PRIX32 " -- received 0x%08" PRIX32 "\r\n", static_cast<unsigned int>(i+j), pattern, ((uint32_t *)buf)[j]);
                mismatch_cnt++;
            }
        }
        //update a status bar every now and again
        if (++cur_step % print_threshold == 0)
        {
            printf("(%.3f%%)\r\n", ((float)cur_step * 100 / (float)num_steps));
        }
    }
    return mismatch_cnt;
}

int FlashLogHarness::test_writeAtLargeAddress(){

    printf("Chip initialized. \r\n");
    printf("MAX SIZE: %" PRIx64 "\r\n", logEnd);
    uint32_t buf = 0xdeadbeef;
    harnessBlockDev->program(&buf, 0x03FFFFF0, 4);
    uint32_t frame;
    harnessBlockDev->read(&frame, 0x03FFFFF0, 4);
    printf("Read %08" PRIx32 "\r\n", frame);
    return 0;
}

/* If SPIFBlockDevice is not defined anywhere, forward-declare it */
class SPIFBlockDevice;

template <typename T>
int FlashLogHarness::test_bulkErase()
{
    /* Check if the type of the block device is a SPIFBlockDevice */
    constexpr bool IS_SPIFBLOCKDEVICE = std::is_same<SPIFBlockDevice, T>();

    /* Use compile-time checks to see if we can use SPIFBlockDevice::bulk_erase */
    if constexpr (IS_SPIFBLOCKDEVICE)
    {
        T* spiFlashDev = reinterpret_cast<T*>(harnessBlockDev.get());
        spiFlashDev->bulk_erase();
        test_check_erase();
    }
    else
    {
        printf("Bulk erase not implemented!\r\n");
    }

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
        printf("found pre-existing log\r\n");
    }
    else if (err.second == FL_ERROR_BD_INIT)
    {
        printf("SPIF driver problem. exiting.\r\n");
        return -1;
    }
    else 
    {
        printf("Log empty.\r\n");
    }
    printf("Writing bad data starting at 0x%" PRIx64 "\r\n", nextPacketAddr);
    harnessBlockDev->program(badDataString, nextPacketAddr, sizeof(badDataString) - 1);
    printf("Bad packet written. Please power cycle and run brownout_recovery test\r\n");
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
        printf("\r\n\nHamster Flash Test Suite:\r\n");

        //MENU. ADD AN OPTION FOR EACH TEST.
        printf("Select a test: \n\r");
        printf("1.  Exit test suite\r\n");
        printf("2.  Chip Erase\r\n");
        printf("3.  Chip Write Pattern\r\n");
        printf("4.  Chip Check Pattern\r\n");
        printf("5.  Chip Write Packets\r\n");
        printf("6.  Chip Check Packets\r\n");
        printf("7.  Chip Iterate Packets\r\n");
        printf("8.  Write Bad Packet\r\n");
        printf("9.  Brownout Recovery\r\n");
        printf("10. Confirm Brownout Recovery\r\n");
        printf("11. Wipe Log\r\n");
        printf("12. Dump Hex Data\r\n");
        printf("13. Dump Binary Data\r\n");
        printf("14. Chip Write Pattern through Log\r\n");
        printf("15. Test checksum\r\n");
        printf("16. Check Erase\r\n");
        printf("17. Write at Large Address\r\n");
        printf("18. Bulk Erase\r\n");
        printf("19. Write Magic Bad Packet\r\n");
        printf("20. Wipe Log (Dirty Part Only)\r\n");

        scanf("%d", &test);
        printf("Running test %d:\r\n\n", test);

        // Clean up garbage characters
        getc(stdin);

        //SWITCH. ADD A CASE FOR EACH TEST.
        switch(test)
        {
            case 1:         printf("Exiting test suite.\r\n");            return 0;
            case 2:         harness.test_chip_erase();                    break;
            case 3:         harness.test_chip_write_pattern();            break;
            case 4:         harness.test_chip_check_pattern();            break;
            case 5:         harness.test_chip_write_packets();            break;
            case 6:         harness.test_chip_read_packets();             break;
            case 7:         harness.test_chip_iterate_packets();          break;
            case 8:         harness.test_write_bad_packet();              break;
            case 9:         harness.test_brownout_recovery();             break;
            case 10:        harness.test_brownout_recovery_confirm();     break;
            case 11:        harness.test_wipe_log(true);                  break;
            case 12:        harness.test_dump_hex();                      break;
            case 13:        harness.test_dump_binary();                   break;
            case 14:        harness.test_chip_write_pattern_through_log();break;
            case 15:        harness.test_checksum();                      break;
            case 16:        harness.test_check_erase();                   break;
            case 17:        harness.test_writeAtLargeAddress();           break;
            case 18:        harness.test_bulkErase();                     break;
            case 19:        harness.test_avoid_partial_packet_tail();     break;
            case 20:        harness.test_wipe_log(false);                 break;
            default:        printf("Invalid test number. Please run again.\r\n"); return 1;
        }

        printf("done.\r\n");
    }
}


#pragma clang diagnostic pop
