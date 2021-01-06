/**
 * @file FlashLog.h
 * @author Conor Hayes, Ian Heidenberger, Skye Mceowen
 * @date 3-17-2018
 * @brief File containing FlashLog packet storage class
 * 
 * Class to provide binary data output buffering and writing to flash chip using a NOR flash library
 * Nearly guarantees recovery of data over random power outages. (some terms and restrictions apply)
 */

#ifndef FLASHLOG_H
#define FLASHLOG_H

#include <inttypes.h>

#include "Packet.h"
#include "States.h"
#include "mbed.h"
#include "SDBlockDevice.h"

#define FLASH_START_ADDR 		0x000000000
#define FLASH_END_ADDR 			0x200000000 //for an 8GB microSD Card

#define FLASH_SECTOR_SIZE 	0x40000

//it's nice to test with a shorter log so the tests don't take ages.
#define LOG_START_ADDR 			FLASH_START_ADDR

#if HALVE_FLASHLOG					
#define LOG_END_ADDR 				(FLASH_END_ADDR/2)
#else //DEBUG-BUILD
#define LOG_END_ADDR				FLASH_END_ADDR
#endif

#define LOG_CAPACITY						(LOG_END_ADDR - LOG_START_ADDR)

// how far backward restoreFSMState() will look for a valid packet to read state info from
#define MAX_PACKETS_TO_CHECK 10

#define SUCCESS 0
#define ERROR_BOUNDS -1
#ifdef ERROR_EMPTY
	#undef ERROR_EMPTY
#endif
#define ERROR_EMPTY -2
#define ERROR_CHECKSUM -3
#define ERROR_TYPE -4
#define ERROR_NOTAIL -5
#define ERROR_LOGNOINIT -6
#define ERROR_FSM_NOT_RESTORED -7
#define ERROR_ITERATION_DONE -8
#define ERROR_LOG_EXISTS -9 //not exactly a warning--just make sure to restore the FSM
#define ERROR_SDBD_INIT -10

#define SD_BLOCK_SIZE 512

/**
 * @brief      Macro for easily iterating through a binary dump
 *						 To be used like:
 *						 ```
 *						 FlashLog log(pin, pin);
 *						 struct log_binary_dump_frame frame;
 *						 FLASHLOG_BINARY_ITERATE(&log, frame)
 *						 {
 *						 		//frame is now filled with data. Do stuff with it
 *						 }
 *						 //the log has now been fully iterated through
 *
 * @param      FRAME  A log_binary_dump_frame instance
 *
 * @return     
 */
#define FLASHLOG_BINARY_ITERATE(FLASHLOG_PTR, FRAME) \
				for (int ITERATION_STATUS = (FLASHLOG_PTR)->binaryDumpIterator(&FRAME, true); \
						 ITERATION_STATUS != ERROR_ITERATION_DONE;	\
						 ITERATION_STATUS = (FLASHLOG_PTR)->binaryDumpIterator(&FRAME, false))

/**
 * @brief      Gets the length of a given packet type
 *
 * @param[in]  type  The packet type
 *
 * @return     Length of the packet in bytes
 */
size_t getPacketLen(uint8_t type);

/**
 * @brief      Finds the location of a packet tail, if any, inside a given buffer
 *
 * @param[in]  buf      The input buffer
 * @param[in]  len      The length of the input buffer
 * @param[in]  reverse  If false, the first packet tail is found. If true, the last packet tail in buf is found
 *
 * @return     Distance of the tail's start from the start of buf, in bytes
 */
int findTailInBuffer(uint8_t *buf, size_t len, bool reverse=false);

/**
 * @brief      Determines if a given buffer is a packet tail.
 *
 * @param      buf   The buffer
 *
 * @return     True if tail, False otherwise.
 */
bool isTail(void *buf);

uint32_t crc32_for_byte(uint32_t r);
void crc32(const void *data, size_t n_bytes, uint32_t* crc);

/**
 * @brief      Class for storing sensor data packets in non-volatile flash memory
 */
class FlashLog
{

public:

	/**
	 * @brief      Constructs the object.
	 *
	 * @param      power   pointer to the power-on timer
	 * @param      flight  pointer to the flight timer
     * @param      flashlogTimer pointer to the flashlog timer
	 * @param      s       Pointer to the HAMSTER FSM state
	 */
	FlashLog(Timer *power, Timer *flight, Timer *flashlogTimer, const enum State *s);

	/**
	 * @brief      Destroys the object.
	 */
	~FlashLog();

	/**
	 * @brief      Initialize the log upon power-on
	 *
	 * @return     true if log is non-empty; false if empty
	 */
	int initLog();

	/**
	 * @brief      Sets the power timer, flight timer, and FSM state to their last recorded values
	 *
	 * @return     SUCCESS, or an ERROR 
	 */
	int restoreFSMState(State *s, uint64_t *pwr_ctr, uint64_t *flight_ctr);

	/**
	 * @brief      Writes a packet to the log
	 *
	 * @param[in]  type  The packet type
	 * @param[in]  data  Buffer containing the packet's bits
	 *
	 * @return     { description_of_the_return_value }
	 */
	int writePacket(uint8_t type, void *data); 

	/**
	 * @brief      Erase the contents of the log
	 * @param[in]  complete  if false, will only erase uptill the last packet
	 * @return     sdBlockDev error
	 */
	int wipeLog(bool complete=true);

	/**
	 * @brief      Reads the next `sizeof(log_binary_dump_frame)` bytes from the memory.
	 *
	 * @param      log_binary_dump_frame  The log binary dump frame
	 * @param[in]  begin                  If true, start from the log's start address.
	 * 									  If false, continue where you left off.
	 *
	 * @return     SUCCESS                  - read successful, call again
	 *             ERROR_ITERATION_DONE     - end of log reached, throw away this packet and do not call again
	 */
	int binaryDumpIterator(struct log_binary_dump_frame *frame, bool begin);

	/**
	 * @brief      Reads the next `sizeof(log_binary_dump_frame)` bytes from the memory in reverse.
	 *
	 * @param      log_binary_dump_frame  The log binary dump frame. stores it in reverse
	 * @param[in]  begin                  If true, start from the log's last written address.
	 *                                    If false, continue where you left off.
	 *
	 * @return     SUCCESS  				- read successful, call again
	 *             ERROR_ITERATION_DONE     - end of log reached, throw away this packet and do not call again
	 */
	int binaryDumpReverseIterator(struct log_binary_dump_frame *frame, bool begin);



	/**
	 * @brief      Gets the total amount of flash memory available to the log
	 *
	 * @return     Log capacity in bytes
	 */
	bd_addr_t getLogCapacity();

	/**
	 * @brief      Gets the amount of flash memory already filled by FlashLog packets
	 *
	 * @return     The log size in bytes
	 */
	bd_addr_t getLogSize();

	/**
	 * @brief      Gets the start address of the FlashLog in the SDBlockDevice's memory space
	 *
	 * @return     The log start address.
	 */
	bd_addr_t getLogStartAddress();

	/**
	 * @brief      Gets the last tail address written in the log
	 *
	 * @return     the last tail address in the log.
	 */
	bd_addr_t getLastTailAddr();

	/**
	 * @brief read tail at provided address into provided buffer
	 * 
	 * @return one of [SUCCESS, ERROR_EMPTY]
	 */
	int readTailAt(bd_addr_t addr, struct packet_tail* buf);

	/**
	 * @brief check whether the provided tail and packet including the tail are valid together
	 * 
	 * @return
	 */
	bool tailDescribesValidPacket(struct packet_tail * checkingTail, bd_addr_t tailAddr);

	/**
	 * @brief Find a packet tail immediately before the provided tail that describes a valid packet, 
	 *        and fill the provided buffers.
	 *        If curPacketTail is not valid, the function will search byte by byte from 
	 *        curPacketTailAddr until it find a valid tail.
	 *        If curPacketTail is valid, and the tail at 
	 *        (curPacketTailAddr - getPacketLen(curPacketTail.typeID)) is valid, then that will be returned
	 *        Else, the function will step backwards byte by byte starting at
	 *        (curPacketTailAddr - getPacketLen(curPacketTail.typeID)) until it finds a valid tail.
	 *
	 * @param[in]  curPacketTailAddr : address of the curPacketTail
	 * @param[in]  curPacketTail     : buffer containing a packet tail, read from flashlog at curPacketTailAddr.
	 *                                 It is okay if this is not a valid packet tail or the tail does not describe a valid packet. 
	 * @param[out] prevPacketTailAddr: address of closest packet tail before curPacketTailAddr that describes a valid packet.
	 * @param[out] prevPacketTail    : buffer containing the packet tail at prevPacketTailAddr.
	 *                                 Guaranteed to descibe a valid packet if @return is SUCCESS 
	 * @return one of [SUCCESS, ERROR_NOTAIL]
	 */
  bd_addr_t findPacketTailBefore(bd_addr_t curPacketTailAddr, struct packet_tail * curPacketTail, bd_addr_t * prevPacketTailAddr, struct packet_tail * prevPacketTail);


  /**
	* @brief Look through the flash log and find NUM_PACKETS_TO_FIND of each sensor packet type.
	*        Print out the average data rate for each packet type. 
  */
  void printPacketsReport(const uint8_t NUM_PACKETS_TO_FIND);


protected:

	/**
	 * DEBUGGING FUNCTIONS. Protected so we can access them from the
	 * FlashLogHarness
	 */

	/**
	 * @brief      Reads the next packet from the log into a buffer
	 *
	 * @param[out] type   The type of the packet we just read
	 * @param      buf    The buffer into which we'll read the packet's bytes
	 * @param[in]  begin  If true, start @ log's first packet. If false, continue where we left off.
	 *
	 * @return     SUCCESS or an ERROR
	 */
	int packetIterator(uint8_t *type, void *buf, bool begin);

	/**
	 * @brief      DEBUG ONLY. Reads the last packet in the log.
	 *
	 * @param[in]  type  The type of the packet we just read
	 * @param      buf   The buffer into which we'll read the packet's bytes
	 *
	 * @return     SUCCESS or ERROR
	 */
	int readLastPacket(uint8_t type, void *buf);

	/**
	 * @brief      Prints the last N bytes in the log
	 *
	 * @param[in]  nBytesPre number of bytes to print AFTER nextPacketAddress
	 * @param[in]  nBytesPost number of bytes to print BEFORE nextPacketAddress
	 */
	void printLastNBytes(size_t nBytesPre, size_t nBytesPost);

	/**
	 * @brief      Return a pointer to the beginning of a packet, given its tail
	 *
	 * @param      tail  The tail
	 *
	 * @return     pointer to the beginning of the packet
	 */
	void *tailToPacket(struct packet_tail* tail);

    /**
     * @brief Write to timeout buffer, flushing if necessary
     *
     * @param buffer Buffer of data to write to blocks
     * @param addr Address of block to begin writing to
     * @param size Size to write in bytes
     *
     * @return BD_ERROR_OK(0) - success SD_BLOCK_DEVICE_ERROR_NO_DEVICE -
     * device (SD card) is missing or not connected
     * SD_BLOCK_DEVICE_ERROR_CRC - crc error SD_BLOCK_DEVICE_ERROR_PARAMETER
     * - invalid parameter SD_BLOCK_DEVICE_ERROR_UNSUPPORTED - unsupported
     * command SD_BLOCK_DEVICE_ERROR_NO_INIT - device is not initialized
     * SD_BLOCK_DEVICE_ERROR_WRITE - SPI write error
     * SD_BLOCK_DEVICE_ERROR_ERASE - erase error
     */
    int writeToLog(const void *buffer, bd_addr_t addr, bd_size_t size);

    /**
     * @brief Read from log to buffer, copying to temporary buffer
     * intermediately
     *
     * @param buffer Buffer to write blocks to
     * @param addr Address of block to begin reading from
     * @param size Size to read in bytes
     *
     * @return BD_ERROR_OK(0) - success SD_BLOCK_DEVICE_ERROR_NO_DEVICE - device
     * (SD card) is missing or not connected SD_BLOCK_DEVICE_ERROR_CRC - crc
     * error SD_BLOCK_DEVICE_ERROR_PARAMETER - invalid parameter
     * SD_BLOCK_DEVICE_ERROR_NO_RESPONSE - no response from device
     * SD_BLOCK_DEVICE_ERROR_UNSUPPORTED - unsupported command
     */
    int readFromLog(void *buffer, bd_addr_t addr, bd_size_t size);

    /**
     * The flash chip is a block device, and FlashLog is built on top of SDBlockDevice
     */
	SDBlockDevice sdBlockDev;
	bd_addr_t nextPacketAddr;		//next empty address in the log
	bd_addr_t nextPacketToRead;	//next packet in a read-through of the log

// private:
    Timer *powerTimerPtr;			// pointer to the HAMSTER power timer
    Timer *flightTimerPtr;		// pointer to the HAMSTER flight timer
    Timer *flashlogTimerPtr; // pointer to SD card timeout timer
    const enum State *sPtr;						// pointer to the HAMSTER fsm state

    bd_addr_t lastTailAddr;			// address of the last tail in the log
    uint8_t lastPacketType;			// type of the last packet in the log
    bool logInitialized;	//set upon successful call of initLog
    int packetsWritten;		//this is wrong after a brownout, so don't depend on it


	//buffer that only contains zeros to allow the SD Card to be overwritten with zeros
	uint8_t zeroBuffer[SD_BLOCK_SIZE] = {0};
    // buffer that fills before timeout to allow writing in blocks
    uint8_t timeoutBuffer[SD_BLOCK_SIZE] = {0};
    // address of the last write to get difference for positioning inside
    // timeoutBuffer
    bd_addr_t lastWrite = UINT64_MAX;
    // address of first write inside timeoutBuffer for call to
    // SDBlockDevice::program
    bd_addr_t firstWritePosition = 0;

    /**
	 * @brief      Returns true if the log is non-empty
	 *
	 * @param      err[out]   sdblockdevice error
	 *
	 * @return     SUCCESS or ERROR
	 */
	bool logExists(int *err);		

	/**
	 * @brief      Uses binary search to find the last packet in the log
	 *
	 * @return     SUCCESS or ERROR
	 */
	int findLastPacket();		

	/**
	 * @brief      Populates the tail of a given packet with the proper data
	 *
	 * @param[in]  type    The packet type
	 * @param[in]  len     The packet length
	 * @param      packet  A buffer containing the packet
	 */
	void populatePacketTail(uint8_t type, size_t len, void *packet);

	/**
	 * @brief      Calculates the checksum for a given packet
	 *
	 * @param[in]  len     The length of the packet
	 * @param      packet  Buffer containing the packet
	 *
	 * @return     The packet's checksum.
	 */
	checksum_t calculateChecksum(size_t len, void *packet);
};

#endif 

