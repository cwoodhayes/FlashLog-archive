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
#include <FlashLogConfig.h>

#include <mbed.h>
#include <BlockDevice.h>

// how far backward restoreFSMState() will look for a valid packet to read state info from
#define MAX_PACKETS_TO_CHECK 10

enum FLResultCode
{
	FL_SUCCESS = 0,

	FL_ITERATION_DONE = -8, // Indicates that this iterator has reached the end

	FL_ERROR_BOUNDS = -1, // Log is full and/or the current operation would go out of bounds
	FL_ERROR_EMPTY = -2, // Indicates that the flashlog is empty
	FL_ERROR_CHECKSUM = -3,
	FL_ERROR_TYPE  = -4, // unrecognized packet type encountered
	FL_ERROR_NOTAIL = -5,
	FL_ERROR_LOGNOINIT = -6, // Log did not init successfully earlier, so can't perform this operation
	FL_ERROR_FSM_NOT_RESTORED = -7, // Failed to find a valid packet from which to restore state.
	FL_ERROR_LOG_EXISTS = -9, // Indicates that the flashlog is not empty but the last packet could not be found
	FL_ERROR_BD_INIT = -10, // Error initializing block device
	FL_ERROR_BD_IO = -11, // Error reading to or writing from block device
	FL_ERROR_BD_PARAMS = -12, // Error with parameters/configuration of block device
};

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
						 ITERATION_STATUS != FL_ITERATION_DONE;	\
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
	 * @param      _blockDev   Block device to use to store data
	 */
	FlashLog(BlockDevice & _blockDev);

	/**
	 * @brief      Destroys the object.
	 */
	~FlashLog();

	/**
	 * @brief      Initialize the log upon power-on,
	 * Also initializes the underlying block devuce
	 *
	 * @return     Pair of FlashLog error code and block device error code.
	 * First element is BD_ERROR_OK if no block device error, and the BD error code otherwise.
	 * Second element is FL_SUCCESS if no error, FL_ERROR_EMPTY if empty (this is not an actual error), or the FlashLog error code otherwise.
	 */
	std::pair<bd_error, FLResultCode> initLog();

	/**
	 * @brief      Sets the power timer, flight timer, and FSM state to their last recorded values
	 *
	 * @return     SUCCESS, or an ERROR 
	 */
	FLResultCode restoreFSMState(FlashLogConfig::State_t *s, ptimer_t *pwr_ctr, ptimer_t *flight_ctr);

	/**
	 * @brief      Writes a packet to the log
	 *
	 * @param[in]  type  The packet type
	 * @param[in]  packet  Buffer containing the packet's bits
	 * @param pwr_ctr Current power counter
	 * @param flight_ctr Current flight counter
	 * @param state Current state of the application
	 *
	 * @return     { description_of_the_return_value }
	 */
	int writePacket(uint8_t type, void *packet, ptimer_t pwr_ctr, ptimer_t flight_ctr, FlashLogConfig::State_t state);

	/**
	 * @brief      Erase the contents of the log
	 * @param[in]  complete  if false, will only erase uptill the last packet.  Otherwise, erases entire chip.
	 *   Theoretically, it should be safe to erase any valid FlashLog with complete=false.  However, if data on
	 *   the block device was corrupted in such a way that there's a large block of 0xFFs and then unerased
	 *   sectors after that, FlashLog can fail to detect the end of the log.  So, if data might have been
	 *   corrupted, you should use complete=true at the cost of much slower speed.
	 * @param progressCallback Callback called when a block is erased.  Argument is the current progress percent
	 *   of the erase operation from 0.0 to 100.0.
	 * @return     sdBlockDev error
	 */
	int wipeLog(bool complete, Callback<void(float)> progressCallback);

    /**
     * @brief      Reads the next `sizeof(log_binary_dump_frame)` bytes from the memory.
     *
     * @param      frame  The log binary dump frame
     * @param[in]  begin  If true, start from the log's start address.
     * 					  If false, continue where you left off.
     *
     * @return     SUCCESS        - read successful, call again
     *             ITERATION_DONE - end of log reached, throw away this packet and do not
     * call again
     */
    FLResultCode binaryDumpIterator(struct log_binary_dump_frame* frame, bool begin);

    /**
     * @brief      Reads the next `sizeof(log_binary_dump_frame)` bytes from the memory in reverse.
     *
     * @param      frame The log binary dump frame. stores it in reverse
     * @param[in]  begin If true, start from the log's last written address.
     *                   If false, continue where you left off.
     *
     * @return SUCCESS        - read successful, call again
     *         ITERATION_DONE - end of log reached, throw away this packet and do not call again
     */
    FLResultCode binaryDumpReverseIterator(struct log_binary_dump_frame* frame, bool begin);

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
	FLResultCode readTailAt(bd_addr_t addr, struct packet_tail* buf);

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
	FLResultCode packetIterator(uint8_t *type, void *buf, bool begin);

	/**
	 * @brief      DEBUG ONLY. Reads the last packet in the log.
	 *
	 * @param[in]  type  The type of the packet we just read
	 * @param      buf   The buffer into which we'll read the packet's bytes
	 *
	 * @return     SUCCESS or ERROR
	 */
	FLResultCode readLastPacket(uint8_t type, void *buf);

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
     * @brief Read from log to buffer.
     * Does NOT need to be aligned in any way.
     * If an error is returned the data in the buffer is undefined.
     *
     * @param buffer Buffer to write blocks to
     * @param addr Address of block to begin reading from
     * @param size Size to read in bytes
     *
     * @return FL_SUCCESS or error code
     */
	FLResultCode readFromLog(void *buffer, bd_addr_t addr, bd_size_t size);

    /**
     * The flash chip is a block device, and FlashLog is built on top of the block device
     */
	BlockDevice & blockDev;

	// The addresses that the FlashLog runs between on the flash device
	bd_addr_t logStart;
	bd_addr_t logEnd; // 1 greater than the largest accessible address
	bd_size_t blockSize; // Reading and writing must be done in blocks of at least this size
	bd_size_t eraseBlockSize; // Erasing must be done in blocks of at least this size

	bd_addr_t nextPacketAddr;		//next empty address in the log
	bd_addr_t nextPacketToRead;	//next packet in a read-through of the log

// private:
    Timer flashlogTimer; // pointer to SD card timeout timer

    bd_addr_t lastTailAddr;			// address of the last tail in the log
    uint8_t lastPacketType;			// type of the last packet in the log
    bool logInitialized;	//set upon successful call of initLog
    int packetsWritten;		//this is wrong after a brownout, so don't depend on it

    // buffer that fills before timeout to allow writing in blocks
    uint8_t writeCache[FlashLogConfig::maxBlockSize];

    // true if a block is currently being cached in the write cache.
    // If false, no cache data is valid.
    bool writeCacheValid = false;

    bd_addr_t cacheBlockAddress; // Sector address that the writeCache currently is caching

    bool firstWriteToLog = true;

	// Buffer needs to be allocated as the max possible searchBlockSize, plus one extra MAX_PACKET_LEN
#define BINARY_SEARCH_BUFFER_SIZE ((MAX_PACKET_LEN + FlashLogConfig::maxBlockSize - 1)/FlashLogConfig::maxBlockSize * FlashLogConfig::maxBlockSize + MAX_PACKET_LEN)

	/**
	 * The scratch buffer is used by a number of different functions that need scratch space.
     * Its contents are undefined before the function is called.
     * The size is the larger of the binary search buffer size (which has its own formula),
     * and one sector of the memory.
     *
     * Current functions using the scratch buffer (calling these will clobber it):
     * - findLastPacket()
     * - wipeLog()
     * - tailDescribesValidPacket()
	 */
    uint8_t scratchBuffer[BINARY_SEARCH_BUFFER_SIZE > FlashLogConfig::maxBlockSize ? BINARY_SEARCH_BUFFER_SIZE : FlashLogConfig::maxBlockSize];

	/** Returns true if a log already exists on the chip.

	 Currently, it just checks the first 90 bytes for any zero bits (erased sectors are always 0xFF).
	 Might want to give it an error tolerance of a bit flip or two, just in case.

	 @param      err   The error encountered when reading the log data, if any

	 @return     true if log exists
	*/
	bool logExists(FLResultCode *err);

	/** Uses binary search to find the last packet written, and then populate the
	 lastTailAddr, nextPacketAddr, and lastPacketType data members.

	 @return     error or success
	*/
	FLResultCode findLastPacket();

    /**
     * @brief      Populates the tail of a given packet with the proper data
     *
     * @param[in] type       The packet type
     * @param[in] len        The packet length
     * @param     packet     A buffer containing the packet
     * @param[in] pwr_ctr    The current time in the power counter
     * @param[in] flight_ctr The current time in the flight counter
     * @param[in] state      The state sending the request
     */
    void populatePacketTail(uint8_t type, size_t len, void* packet, ptimer_t pwr_ctr,
        ptimer_t flight_ctr, FlashLogConfig::State_t state);

    /**
	 * @brief      Calculates the checksum for a given packet
	 *
	 * @param[in]  len     The length of the packet
	 * @param      packet  Buffer containing the packet
	 *
	 * @return     The packet's checksum.
	 */
	checksum_t calculateChecksum(size_t len, void *packet);

	/**
	 * Round the given address down to the nearest block
	 * start address.
	 * If given a block start address, returns it unchanged.
	 * @param address
	 */
	inline bd_addr_t roundDownToNearestBlock(bd_addr_t address)
	{
		return (address / blockSize) * blockSize;
	}

	/**
	 * Round the given address up to the nearest block
	 * start address.
	 * If given a block start address, returns it unchanged.
	 * @param address
	 */
	inline bd_addr_t roundUpToNearestBlock(bd_addr_t address)
	{
		return ((address + blockSize - 1) / blockSize) * blockSize;
	}

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
	 * Clear all stored information in the write cache, and prepare for it to load a new block for writing.
	 */
	void clearWriteCache();

};

#endif 
