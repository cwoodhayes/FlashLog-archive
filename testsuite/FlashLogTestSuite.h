#pragma once

#include <chrono>

#include <FlashLog.h>

namespace chrono = std::chrono;

class FlashLogHarness : public FlashLog
{
	public:

		FlashLogHarness();

		/**
		 * @brief      Erase the chip. Then check that it was erased
		 *
		 * @return     error
		 */
		int test_chip_erase();

		/**
		 * @brief      Write a pattern across the chip using the block device itself
		 *
		 * @return     error
		 */
		int test_chip_write_pattern();

		/**
		* @brief      Write a pattern across the chip using writeToLog()
		*
		* @return     error
		*/
		int test_chip_write_pattern_through_log();

		/**
		 * @brief      Confirm that the pattern written in write_pattern was indeed
		 * 						 written across the chip.
		 *
		 * @return     error
		 */
		int test_chip_check_pattern();

		/**
		 * @brief      Log a repeating pattern of packets using FlashLog until
		 * the memory is about half full. 
		 *
		 * @return     error
		 */
		int test_chip_write_packets();

		/**
		 * @brief      Use the flash block device driver directly to
		 * 						 confirm that the repeating pattern of packets
		 *  					 was written to the flash accurately by chip_write_packets. 
		 *  					 Print out any mismatches.
		 *
		 * @return     error
		 */
		int test_chip_read_packets();

		/**
		 * @brief      Use the FlashLog's readNextPacket function to iterate through 
		 * 						 the log, checking to seethat the previous call to chip_write_packets 
		 * 						 wrote each packet as expected.For each packet in the log, prints an error
		 * 						 if it's wrong, and nothing if it's right
		 *
		 * @return     error
		 */
		int test_chip_iterate_packets();

		/**
		 * @brief      Manually write a bad packet to the end of the flash using SPIFBlockDevice
		 * 						 After running this, confirm that the log can recover by running brownout_recovery 
		 *
		 * @return     error
		 */
		int test_write_bad_packet();

		/**
		 * @brief      Use FlashLog initLog() to recover from the power cycle by recovering program
		 * 						 fsmState and correcting the timer values
		 *
		 * @return     error
		 */
		int test_brownout_recovery();

		/**
		 * @brief      Tests that the sequence of chip_write_packets->brownout_recovery has 
		 * 						 written packets to the memory in the manner we expect by reading back
		 * 						 all the packets.
		 *
		 * @return     error
		 */
		int test_brownout_recovery_confirm();

		/**
		 * @brief      Wipe the entirety of the log
		 *
		 * @return     err
		 */
		int test_wipe_log();

		/**
		 * @brief      Dump the entirety of the log's bytes in ascii-encoded hex
		 * @return     err
		 */
		int test_dump_hex();

		/**
		 * @brief      Dump the entirety of the log's bytes in raw binary
		 * @return     err
		 */
		int test_dump_binary(Stream &pc);

		/**
		 * @brief      Check that the log is empty
		 *             by running a loop checking for zeroes in the memory, and also
		 *             by running initLog and confirming that the log is empty
		 *
		 * @return     # of bad bytes
		 */
		int test_check_erase();

		/**
		 * @brief      Confirm that the flashlog's checksum calculator is working
		 *
		 * @return     err
		 */
		int test_checksum();

		/**
		 * @brief      Confirm that the flashlog's checksum calculator is working
		 *
		 * @return     err
		 */
		int test_bulkErase();
		
		/**
		 * @brief      Confirm that 32 bit addressing works
		 *
		 * @return     err
		 */
		int test_writeAtLargeAddress();

		int test_avoid_partial_packet_tail();


	private:

		/* Global variables for use in all test functions: */
		Timer flightTimer;
		Timer powerTimer;
		FlashLogConfig::State_t fsmState = FlashLogConfig::harnessExampleState;

		// Offsets to store restored timer values
		ptimer_t powerTimerOffset = 0;
		ptimer_t flightTimerOffset = 0;

		/**
		 * @brief      Start the fake flight + power timers
		 */
		void startTimers(uint64_t offset=0);

		/**
		 * Get the power timer value in microseconds, factoring in the offset
		 * @return
		 */
		ptimer_t getPowerTimer()
		{
			return chrono::duration_cast<chrono::microseconds>(powerTimer.elapsed_time()).count() + powerTimerOffset;
		}

		/**
		 * Get the flight timer value in microseconds, factoring in the offset
		 * @return
		 */
		ptimer_t getFlightTimer()
		{
			return chrono::duration_cast<chrono::microseconds>(flightTimer.elapsed_time()).count() + flightTimerOffset;
		}

        /**
         * @brief      Writes a pattern across the flash memory
         *
         * @param[in]  pattern    32-bit pattern to write
         * @param[in]  start_addr The address at which we'll start writing the pattern
         * @param[in]  len        The length of memory across which to write the pattern, in bytes
         *
         * @return     err
         */
        int write_pattern(uint32_t pattern, bd_addr_t start_addr, bd_size_t len);

        /**
         * @brief      Writes a pattern using FlashLog
         *
         * @param[in]  pattern    32-bit pattern to write
         * @param[in]  start_addr The address at which we'll start writing the pattern
         * @param[in]  len        The length of memory across which to write the pattern, in bytes
         *
         * @return     err
         */
        int write_pattern_through_flashlog(uint32_t pattern, bd_addr_t start_addr, bd_size_t len);

        /**
         * @brief      Checks that a pattern has been correctly written across the memory
         *
         * @param[in]  pattern    32-bit pattern to check for
         * @param[in]  start_addr The address at which we'll start checking the pattern
         * @param[in]  len        The length of memory across which to check the pattern, in bytes
         *
         * @return     0 if the pattern is correctly written, -1 otherwise
         */
        int check_pattern(uint32_t pattern, bd_addr_t start_addr, bd_size_t len);
};

//test packet instantiation
struct log_packet_text tp_text
{
	"Space or Nothing! (TM) #FlightTheFluckOn"
};

struct log_packet_gps tp_gps
{
	2.1,
	2.2,
	20000,
	10,
	3,
	21,
	22,
	2019,
	10,
	22,
	11,
	5,
	3,
	23,
	UBlox::GPSFix::FIX_3D
};

struct log_packet_bno tp_bno
{
	1.2, 3.4, 5.67, 8.9,
	1.111, 2.222, 3.333,
	0, 1, 2,
	'a'
};

struct log_packet_temp tp_temp
{
	55
};

struct log_packet_baro tp_baro
{
	300, 200, 100
};

struct log_packet_accel tp_accel
{
	0x10, //ax
  0x11, //ay
  0x12, //az
};

struct log_packet_power tp_power
{
	12.0, //battVoltage
	50, //chargePercent
	.123, //battCurrent
	.120 //reg5VCurrent
};


#define TP_LIST1_SIZE 7
void *tp_list1[TP_LIST1_SIZE] = {
			&tp_text,
			&tp_gps,
			&tp_bno, 
			&tp_temp, 
			&tp_baro, 
			&tp_accel,
			&tp_power
		};
int tp_list1_types[TP_LIST1_SIZE] = {
			LOG_TEXT,
			LOG_GPS,
			LOG_BNO,
			LOG_TEMP,
			LOG_BARO,
			LOG_ACCEL,
			LOG_POWER
		};

#define TP_LIST2_SIZE 10
void *tp_list2[TP_LIST2_SIZE] = {
			&tp_gps,
			&tp_text,
			&tp_text,
			&tp_bno, 
			&tp_temp, 
			&tp_gps,
			&tp_baro, 
			&tp_accel,
			&tp_text,
			&tp_baro
		};
int tp_list2_types[TP_LIST2_SIZE] = {
			LOG_GPS,
			LOG_TEXT,
			LOG_TEXT,
			LOG_BNO, 
			LOG_TEMP, 
			LOG_GPS,
			LOG_BARO, 
			LOG_ACCEL,
			LOG_TEXT,
			LOG_BARO };
