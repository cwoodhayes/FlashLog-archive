/**
 * @file FlashLog.cpp
 * @author Conor Hayes, Ian Heidenberger, Skye Mceowen, Jamie Smith, Jasper Swallen
 * @date 3-17-2018
 * @brief File containing FlashLog packet storage class
 * 
 * Class to provide binary data output buffering and writing to flash chip using a NOR flash library
 * Nearly guarantees recovery of data over random power outages. (some terms and restrictions apply)
 */

#include <cinttypes>
#include <cstring>

#include "FlashLog.h"

//#define FL_DEBUG

FlashLog::FlashLog(BlockDevice & _blockDev):
blockDev(_blockDev)
{
    logInitialized = false;

    // currently there's no situation where the start address would be nonzero... but we still have a variable for it
    logStart = 0;

    nextPacketAddr = logStart;
    lastTailAddr = logStart;
    packetsWritten = 0;
    lastPacketType = LOG_INVALID;

    nextPacketToRead = logStart;
}

/* Destructor */
FlashLog::~FlashLog() {
    blockDev.deinit();
}

/* Preps a new log by checking to make sure there's no log already written. If
 there's a log already there, find the next address to write to and continue
 that existing log without overwriting it. If there's no log, we can leave
 things as-is.

 @return     true if a log exists in the memory
*/
std::pair<bd_error, FLResultCode> FlashLog::initLog() {

    bd_error blockDevErr = static_cast<bd_error>(blockDev.init());
    if (blockDevErr)
    {
        printf("[FlashLog] Error %d initializing device!", blockDevErr);
        return {blockDevErr, FL_ERROR_BD_INIT};
    }

#ifdef FL_DEBUG
      printf("block device size: 0x%" PRIX64 "\r\n",         blockDev.size());
      printf("block device read size: 0x%" PRIX64 "\r\n",    blockDev.get_read_size());
      printf("block device program size: 0x%" PRIX64 "\r\n", blockDev.get_program_size());
      printf("block device erase size: 0x%" PRIX64 "\r\n",   blockDev.get_erase_size());
#endif

	// detect log size from block device
	logEnd = blockDev.size() / FlashLogConfig::sizeDivisor;

    if(!FlashLogConfig::isSPIFlash)
    {	
        if(blockDev.get_program_size() != blockDev.get_read_size() || blockDev.get_program_size() != blockDev.get_erase_size())
        {
            printf("[FlashLog] Error: Program, erase, and read sizes of the memory must be equal in SD card mode\r\n");
        }
    }

	MBED_ASSERT(blockDev.get_program_size() == blockDev.get_read_size());
	blockSize = blockDev.get_program_size();
	if(blockSize > FlashLogConfig::maxBlockSize)
	{
		printf("[FlashLog] Error: Not compiled with support for memory with block size %" PRIu64 ", please increase FlashLogConfig::maxBlockSize\r\n", blockSize);
		return {BD_ERROR_OK, FL_ERROR_BD_PARAMS};
	}

	eraseBlockSize = blockDev.get_erase_size();

	FLResultCode err;

    // remove any data that may have been in the write cache
	clearWriteCache();

	if (logExists(&err)) {
        err = findLastPacket();
        if (err)
        {
            printf("[FlashLog] Init err %d\r\n", err);
            return {BD_ERROR_OK, err};
        }
        else
        {
            err = FL_ERROR_LOG_EXISTS;
        }
    }
    else
    {
        err = FL_SUCCESS;
    }
    this->logInitialized = true;

	printf("[FlashLog]: Log initialized from 0x%016" PRIX64 "-0x%016" PRIX64 ".\r\n",
                        getLogStartAddress(), getLogStartAddress()+getLogCapacity());
    printf("[FlashLog]: Log is currently %.1f%% filled (%" PRIu64 " bytes).\r\n",
                        (float) getLogSize() * 100.0 / (float) getLogCapacity(), getLogSize());
    return {BD_ERROR_OK, err};
}


bool FlashLog::logExists(FLResultCode *err) {
    uint8_t buf[MAX_PACKET_LEN];
    *err = readFromLog(buf, logStart, MAX_PACKET_LEN);
    for (size_t i=0; i<MAX_PACKET_LEN; i++) {
        if (buf[i] != 0xFF){
            // printf("i:%" PRIu32 " c:%x %x\r\n",i,buf[i], ~buf[i]);
            return true;
        }
    }
    return false;
}

/* Uses binary search to find the last packet written, and then populate the
 lastTailAddr, nextPacketAddr, and lastPacketType data members.

 @return     error
*/
FLResultCode FlashLog::findLastPacket() {
    // If the block size is > 2*max, then we need to search based on that in order to handle blanks of up to SD_BLOCK_SIZE - 1
    // Otherwise, needs to be 2*max rounded up to the nearest block for when we find the END of the last packet.
	const size_t searchBlockSize = (MAX_PACKET_LEN + blockSize - 1)/blockSize * blockSize;

    FLResultCode err = FL_SUCCESS;
    bd_addr_t start=logStart, end=logEnd, middle=logEnd - searchBlockSize;

	MBED_ASSERT(end - start >= 2 * searchBlockSize); // make sure that the loop is actually entered

    //binary search until we're somewhere inside the last packet
    bool isPacket = false;
    while ( static_cast<bd_size_t>(end - start) >= 2*searchBlockSize) {

    	//here be packets? yarr?

        // We will check searchBlockSize bytes starting from the middle of the current memory region, given here.
        middle = start/2+end/2;    //2 divisions to avoid unsigned overflow

        FLResultCode readError = readFromLog(scratchBuffer, middle, searchBlockSize);

        if(readError)
		{
        	err = readError;
		}
        isPacket = false;

        for (size_t i=0; i<searchBlockSize; i++) {    //TODO just use 32 bit operands where possible
            if (scratchBuffer[i] != 0xFF) {
                isPacket = true;
                break;
            }
        }
#ifdef FL_DEBUG
        printf("\tBinary search: checking middle of 0x%016" PRIX64 "-0x%016" PRIX64 " range (0x%016" PRIX64 ") for any packet data\r\n", start, end, middle);
        PRINT_BYTES(scratchBuffer, MAX_PACKET_LEN);
        printf("\r\n");
#endif

        //if we found a packet, search higher addresses. Else, search lower addresses.
        if (isPacket) start = middle;
        else end = middle;
    }

    if(middle == (logEnd - searchBlockSize) && !isPacket){
        printf("Could not find end of log! Log may be full\r\n");
        return FL_ERROR_BOUNDS;
    }


	// The end of the last packet is now somewhere between `start` and `end`.
	// The size of this region is somewhere between 1 and 2 search blocks.
	// (Remember, it must be >=1 block to tolerate up to (blockSize-1) of fill bytes)

    // We know that the end of the last packet is somewhere between start and end.
    // So, expand the region backward to definitely without fail contain the entire last packet.
	bd_addr_t finalRegionStart = (start - logStart < MAX_PACKET_LEN) ? logStart : start-MAX_PACKET_LEN;

	const bd_size_t finalRegionSize = end - finalRegionStart;

	// sanity check
	MBED_ASSERT(sizeof(scratchBuffer) >= finalRegionSize);

	FLResultCode readError = readFromLog(scratchBuffer, finalRegionStart, finalRegionSize);    //fill up the buffer in one shot
	if(readError)
	{
		err = readError;
	}

#ifdef FL_DEBUG
	printf("discovered end of logfile near %016" PRIX64 ". Final packet must exist somewhere in this region:\r\n", start);
    PRETTYPRINT_BYTES(scratchBuffer, finalRegionSize, finalRegionStart);
#endif

	/*Creep along until we haven't seen anything but FF in MAX_PACKET_LEN bytes, then use the last time we saw 0 bits as
	the end of the tail.*/
    bd_addr_t lastDirtyByte =0;
    for (bd_addr_t currAddr = finalRegionStart; currAddr < end; currAddr++) {
        if (scratchBuffer[currAddr - finalRegionStart] != 0xFF) lastDirtyByte = currAddr;
    }
#ifdef FL_DEBUG
    printf("\r\nLast dirty byte found at flash address 0x%016" PRIX64 "\r\n", lastDirtyByte);
#endif

    readFromLog(writeCache, roundDownToNearestBlock(lastDirtyByte), blockSize);
    writeCacheValid = true;

    /*     initLog() doesn't actually fully handle the tailless-packet case. If it discovers that the last
        bytes written to the log are not a valid tail, it concatenates a tail of type LOG_INVALID to those bytes,
        marking them out to be handled by restoreFSMState().
    */

    struct packet_tail possibleTail;

    bool tailValid = true;
    if(lastDirtyByte < sizeof(struct packet_tail) -1){
        tailValid = false;
    }
    
    if(tailValid)
    {
        // try viewing the data as a tail
        memcpy(&possibleTail, scratchBuffer+(lastDirtyByte-finalRegionStart)-sizeof(struct packet_tail)+1, sizeof(struct packet_tail));

        //tailp now points to the first byte in the tail, if this is a valid packet
#ifdef FL_DEBUG
        printf("Hopefully this is a tail. If not, we'll mark it as invalid and handle it in restoreFSMState:");
        PRETTYPRINT_BYTES(&possibleTail, sizeof(struct packet_tail), lastDirtyByte-sizeof(struct packet_tail)+1);
        printf("\r\n");
#endif

        tailValid = isTail(&possibleTail);
    }

    //finally, use the found tail to populate the class data members.
    nextPacketAddr = lastDirtyByte + 1;

    // TODO test with invalid tail
    cacheBlockAddress = roundDownToNearestBlock(nextPacketAddr);

    //if this isn't a valid tail, write a valid tail marking this as an invalid packet and continue
    if (tailValid) 
    {
    	lastPacketType = possibleTail.typeID;
    }
    else
    {
        printf ("invalid tail! Writing tail of type LOG_INVALID starting at 0x%016" PRIX64 ":\r\n", nextPacketAddr);
        //reuse the buffer tailp points to since its data is invalid anyhow
        struct packet_tail invalidTail = {};
        populatePacketTail(LOG_INVALID, sizeof(struct packet_tail), &invalidTail, 0, 0, FlashLogConfig::invalidState);
        printf("tail mbed addr: %p; flash addr: %016" PRIX64 "\r\n", &invalidTail, nextPacketAddr);
        printf("Word of warning: This tail may look gross. But all that matters is magic and packet type. Tail:\r\n");
        PRINT_BYTES(&invalidTail, sizeof(struct packet_tail));
        writeToLog(&invalidTail, nextPacketAddr, sizeof(struct packet_tail));
        nextPacketAddr += sizeof(struct packet_tail);

        //restoreFSMState is responsible for finding the last valid packet
        lastPacketType = invalidTail.typeID;
    }

    lastTailAddr = nextPacketAddr - sizeof(struct packet_tail);
    return err;
}

int FlashLog::writeToLog(const void *buffer, bd_addr_t addr, bd_size_t size)
{
	if(blockSize == 1)
	{
		// byte addressable = easy-peasy
		return blockDev.program(buffer, addr, size);
	}

#ifdef FL_DEBUG
	printf("[writeToLog()] Called to program 0x%" PRIx64 " bytes starting at addr 0x%" PRIx64 "\r\n", size, addr);
#endif

    if(firstWriteToLog)
    {
        flashlogTimer.start();
        firstWriteToLog = false;
    }

    // TODO proper error handling. This just returns *last* error, not
    // all/first/worst/??? error. Also, just returns 0 if buffer is not flushed,
    // so returning 0 does *not* mean successful write, it means the *last*
    // write was successful or there was not a write

    bd_size_t writeStartOffset;
    if(!writeCacheValid)
	{
        cacheBlockAddress = roundDownToNearestBlock(addr);
	}
	writeStartOffset = addr - cacheBlockAddress;

    int err = 0;

    if(writeStartOffset >= blockSize)
    {
        // if the next write is after the end of the block, flush current block,
        // set cacheBlockAddress, and follow logic as expected
#ifdef FL_DEBUG
		printf("[writeToLog(): o] Programming to 0x%" PRIx64 ": ", cacheBlockAddress);
		PRETTYPRINT_BYTES(writeCache, blockSize, cacheBlockAddress);
#endif
        err = blockDev.program(writeCache, cacheBlockAddress, blockSize);

        clearWriteCache();
        flashlogTimer.reset();

        cacheBlockAddress = roundDownToNearestBlock(addr);
        writeStartOffset = addr - cacheBlockAddress;
    }

    if(size + writeStartOffset >= blockSize)
    {
        // space left in first block availble to write to
        bd_size_t leftoverIndex = blockSize - writeStartOffset;
        // write current cache and fill leftover size
        memcpy(&writeCache[writeStartOffset], buffer, leftoverIndex);

#ifdef FL_DEBUG
		printf("[writeToLog(): s+o] Programming to 0x%" PRIx64 ": ", cacheBlockAddress);
		PRETTYPRINT_BYTES(writeCache, blockSize, cacheBlockAddress);
#endif
        err = blockDev.program(writeCache, cacheBlockAddress, blockSize);

		cacheBlockAddress += blockSize;

        bd_size_t leftoverSize = size - leftoverIndex;
        while(leftoverSize > blockSize)
        {
            // writes in chunks of SD_BLOCK_SIZE bytes
#ifdef FL_DEBUG
			printf("[writeToLog(): l] Programming to 0x%" PRIx64 ": ", cacheBlockAddress);
			PRETTYPRINT_BYTES(&(reinterpret_cast<const uint8_t *>(buffer)[leftoverIndex]), blockSize, cacheBlockAddress);
#endif
            err = blockDev.program(&(reinterpret_cast<const uint8_t *>(buffer)[leftoverIndex]),
									 cacheBlockAddress, blockSize);
			cacheBlockAddress += blockSize;
			leftoverIndex += blockSize;
            leftoverSize -= blockSize;
        }

        flashlogTimer.reset();

        clearWriteCache();

        if(leftoverSize > 0)
        {
            // start new writeCache
            MBED_ASSERT(cacheBlockAddress - (addr + size - leftoverSize) == 0);
            memcpy(&writeCache[0], &(reinterpret_cast<const uint8_t *>(buffer)[size - leftoverSize]),
				   leftoverSize);
            writeCacheValid = true;
        }
    }
    else
    {
        memcpy(&writeCache[writeStartOffset], buffer, size);
        writeCacheValid = true;
    }

    if(writeCacheValid && (flashlogTimer.elapsed_time() > FlashLogConfig::cacheTimeout))
    {
    	// Flush write cache to chip
#ifdef FL_DEBUG
		printf("[writeToLog()] Flushing cache: programming to 0x%" PRIx64 ": ", cacheBlockAddress);
		PRETTYPRINT_BYTES(writeCache, blockSize, cacheBlockAddress);
#endif
        err = blockDev.program(writeCache, cacheBlockAddress, blockSize);

        flashlogTimer.reset();
    }

    return err;
}

FLResultCode FlashLog::readFromLog(void *buffer, bd_addr_t addr, bd_size_t size)
{
    int err = 0;
    if(FlashLogConfig::isSPIFlash)
    {
        err = blockDev.read(buffer, addr, size);
        if(err != BD_ERROR_OK)
        {
            return FL_ERROR_BD_IO;
        }
        else
        {
            return FL_SUCCESS;
        }
    }

    static uint8_t readBuffer[FlashLogConfig::maxBlockSize] = {0};
    bd_size_t sizeRemaining = size;
    size_t nextByteIndex = 0; // next index to write to in the buffer
    const bd_addr_t readEnd = addr + size;

	// first read "prologue": segment before start of first full block
	const bd_addr_t prologueStart = addr;
	const bd_addr_t prologueEnd = roundUpToNearestBlock(addr); // first address AFTER the end of the prologue

	if(prologueEnd == prologueStart)
	{
		// read starts on a block-aligned address
	}
	else
	{
		// read block containing prologue
		err = blockDev.read(readBuffer, roundDownToNearestBlock(addr), blockSize);

		if(err != BD_ERROR_OK)
		{
			return FL_ERROR_BD_IO;
		}

		// copy prologue data into buffer
		bd_size_t offsetIntoPrologueBlock = prologueStart - roundDownToNearestBlock(addr);
		MBED_ASSERT(offsetIntoPrologueBlock < blockSize);

		bd_size_t prologueLength = prologueEnd - prologueStart;
		if(prologueLength > sizeRemaining)
		{
			prologueLength = sizeRemaining;
		}

		memcpy(reinterpret_cast<uint8_t *>(buffer) + nextByteIndex, readBuffer + offsetIntoPrologueBlock, prologueLength);
		sizeRemaining -= prologueLength;
		nextByteIndex += prologueLength;
	}

	if(sizeRemaining == 0)
	{
		return FL_SUCCESS;
	}

	// now read "body": segment containing full blocks
	const bd_addr_t bodyStart = prologueEnd;
	bd_addr_t bodyEnd;

	if(sizeRemaining >= blockSize)
	{
		// one or more full blocks can now be read
		bodyEnd = roundDownToNearestBlock(readEnd);
		bd_size_t bodyLength = bodyEnd - bodyStart;

		// read 'em!  Straight into the output buffer.
		err = blockDev.read(reinterpret_cast<uint8_t *>(buffer) + nextByteIndex, bodyStart, bodyLength);
		if(err != BD_ERROR_OK)
		{
			return FL_ERROR_BD_IO;
		}

		sizeRemaining -= bodyLength;
		nextByteIndex += bodyLength;
	}
	else
	{
		// less than one full block remaining
		bodyEnd = bodyStart;
	}

	if(sizeRemaining == 0)
	{
		return FL_SUCCESS;
	}

	// finally, read the "epilogue": the final non-full block at the end of the read
	const bd_addr_t epilogueStart = bodyEnd;
	const bd_addr_t epilogueEnd = readEnd;

	// epilogue should be at least 1 byte long if we have reached here
	MBED_ASSERT(epilogueStart != epilogueEnd);

	// read block containing epilogue
	err = blockDev.read(readBuffer, epilogueStart, blockSize);

	if(err != BD_ERROR_OK)
	{
		return FL_ERROR_BD_IO;
	}

	// copy epilogue data into buffer
	bd_size_t epilogueLength = epilogueEnd - epilogueStart;

	memcpy(reinterpret_cast<uint8_t *>(buffer) + nextByteIndex, readBuffer, sizeRemaining);
	sizeRemaining -= epilogueLength;

	MBED_ASSERT(sizeRemaining == 0);

	return FL_SUCCESS;
}

bool FlashLog::tailDescribesValidPacket(struct packet_tail * checkingTail, bd_addr_t tailAddr){
        // first check if we have the correct magic.
        if(checkingTail->magic1 != LOG_PACKET_MAGIC1
          || checkingTail->magic2 != LOG_PACKET_MAGIC2
          || checkingTail->magic3 != LOG_PACKET_MAGIC3)
        {
#ifdef FL_DEBUG
            printf("Candidate packet has invalid magic numbers.\r\n");
#endif
            return false;
        }

        if(checkingTail->typeID == LOG_INVALID)
        {
            // This is an invalid packet written by initLog() when it finds garbage at the end
            // of the log.  We want to skip it and find the next valid packet.
#ifdef FL_DEBUG
            printf("Candidate packet has type LOG_INVALID.\r\n");
#endif
            return false;
        }

        // if we have gotten this far it at least looks like a packet, but there are still other things to check

        // next check that type is valid (important so we know its length)
        if(checkingTail->typeID >= LOG_END_TYPEID)
        {
#ifdef FL_DEBUG
            printf("Candidate packet has invalid typeID %" PRIu8 ".\r\n", checkingTail->typeID);
            PRETTYPRINT_BYTES(&checkingTail, sizeof(struct packet_tail), tailAddr);
#endif
            return false;
        }

        // now sanity-check that the packet would start after the beginning of the log to
        // avoid the possibility of trying to access a negative address
        bd_addr_t packetStartAddr = 0;
        size_t packetLength = getPacketLen(checkingTail->typeID);
        if(tailAddr < packetLength - sizeof(struct packet_tail) + logStart)
        {
            // protect against unsigned rollover when checking if packetStartAddr < LOG_START_ADDR
#ifdef FL_DEBUG
            printf("Candidate packet has impossible packet type\r\n");
#endif
            return false;
        }
        else
        {
            // now compute address
            packetStartAddr = tailAddr - (packetLength - sizeof(struct packet_tail));
        }

        // finally check checksum (which requires reading the whole packet)
        readFromLog(scratchBuffer, packetStartAddr, packetLength);
        checksum_t packetChecksum = calculateChecksum(packetLength, scratchBuffer);
        if(packetChecksum != checkingTail->checksum)
        {
#ifdef FL_DEBUG
            printf("Candidate packet has invalid checksum.\r\n");
            printf("Calculated Checksum %" PRIx32 "\r\n", packetChecksum);
            printf("Existing Checksum %" PRIx32 "\r\n", checkingTail->checksum);
            PRETTYPRINT_BYTES(&checkingTail, sizeof(struct packet_tail), tailAddr);
#endif
            return false;
        }

    return true;
}


FLResultCode FlashLog::readTailAt(bd_addr_t addr, struct packet_tail* buf){
    if(addr >= logStart && addr <= logEnd - sizeof(struct packet_tail)){
        return readFromLog(buf, addr, sizeof(struct packet_tail));
    }
    else{
        return FL_ERROR_BOUNDS;
    }
}


FLResultCode FlashLog::restoreFSMState(FlashLogConfig::State_t *s, ptimer_t *pwr_ctr, ptimer_t *flight_ctr)
{
    if (!logInitialized) {
        printf("[FlashLog] ERROR: no init (restoreFSMState)\r\n");
        return FL_ERROR_LOGNOINIT;
    }

    if (lastTailAddr == logStart) return FL_ERROR_EMPTY;

    // read initial packet tail, starting at lastTailAddr
    struct packet_tail * restorationTail;
    
    struct packet_tail lastTail = {};
    readFromLog(&lastTail, lastTailAddr, sizeof(struct packet_tail));

    struct packet_tail prevTail = {};

    if(tailDescribesValidPacket(&lastTail, lastTailAddr)){
        restorationTail = &lastTail;
    }
    else{
        bd_addr_t prevTailAddr;
        if(findPacketTailBefore(lastTailAddr, &lastTail, &prevTailAddr, &prevTail) == FL_SUCCESS){
            restorationTail = &prevTail;
        }
        else{
            // checked the maximum distance and found nothing.
            printf("FATAL ERROR: Valid packet not found after %d bytes, FSM State not restored. \r\n", MAX_PACKET_LEN * MAX_PACKETS_TO_CHECK);
            return FL_ERROR_FSM_NOT_RESTORED;
        }
    }

    if(!(FlashLogConfig::isValidState(static_cast<FlashLogConfig::State_t>(restorationTail->state))))
	{
    	// invalid state ID -- maybe data is from a different version of the application?
		printf("FATAL ERROR: Valid packet found but state %" PRIu8 " is not valid. \r\n", restorationTail->state);
		return FL_ERROR_FSM_NOT_RESTORED;
	}

    // we haven't returned due to resoration failure, so go ahead and fill buffers
    *pwr_ctr = restorationTail->pwr_ctr;
    *flight_ctr = restorationTail->flight_ctr;

    // restore FSM state
    *s = static_cast<FlashLogConfig::State_t>(restorationTail->state);

    return FL_SUCCESS;
}


void FlashLog::printPacketsReport(const uint8_t NUM_PACKETS_TO_FIND){
    // Arrays for packets
    const uint8_t numPacketTypes = LOG_END_TYPEID - 1;
    const int MAX_SEARCH_PACKETS = 500;

    uint32_t packetsFound[numPacketTypes];
    int32_t packetDeltaAvg[numPacketTypes];
    ptimer_t lastPacketTime[numPacketTypes];

    // Initialize packetCounter items to 0
    for (int i = 1; i < numPacketTypes + 1; i++) {
        packetsFound[i-1] = 0;
        lastPacketTime[i-1] = 0;
        packetDeltaAvg[i-1] = 0;
    }

    int32_t packetDeltaTime;
    bd_addr_t curTailAddr = getLastTailAddr();
    struct packet_tail curTail = {};
    int numPacketsSearched = 0;

    while (numPacketsSearched < MAX_SEARCH_PACKETS) {

        if(numPacketsSearched == 0){
            if(readTailAt(curTailAddr, &curTail) != FL_SUCCESS){
                break;
            };
        }
        else{
            // push the curTail back one packet
            bd_addr_t nextTailAddr;
            struct packet_tail nextTail;
            if(findPacketTailBefore(curTailAddr, &curTail, &nextTailAddr, &nextTail) == FL_SUCCESS){
                curTailAddr = nextTailAddr;
                curTail = nextTail;
            }
            else{
                break;
            }
        }

        bool searchComplete = true;
        for(int i = 1; i < numPacketTypes + 1; i++){
            if(i == LOG_TEXT || i == LOG_INVALID){ continue; }
            if(packetsFound[i-1] < NUM_PACKETS_TO_FIND){
                searchComplete = false;
            }
        }
        if(searchComplete){
            break;
        }
        
        // Ignore invalid typeIDs.
        if (curTail.typeID == LOG_TEXT || curTail.typeID == LOG_INVALID){ numPacketsSearched++; continue; }

        // Calculate packet delta time and update counters
        packetDeltaTime = (int32_t) (lastPacketTime[curTail.typeID -1] - curTail.pwr_ctr)/1000;
        if (packetsFound[curTail.typeID -1] > 0) {
            // We found a packet of this type before.

            //update average
            if(packetsFound[curTail.typeID -1] >= 2){
                packetDeltaAvg[curTail.typeID-1] = packetDeltaAvg[curTail.typeID-1] + ((packetDeltaTime - packetDeltaAvg[curTail.typeID-1])/((int32_t)packetsFound[curTail.typeID-1]+1));
            }
            else{
                packetDeltaAvg[curTail.typeID-1] = packetDeltaTime;
            }
        }
        // Update the packet's last power counter time.
        lastPacketTime[curTail.typeID-1] = curTail.pwr_ctr;
        // Update count for this type of packet.
        packetsFound[curTail.typeID-1]++;
        numPacketsSearched++;


    }

    // Print out all the packet types and their averages.
    
    printf("\r\n======================================\r\n");
    printf("Last Packet Tail found at: %016" PRIx64 "\r\n", getLastTailAddr());
    printf("Searched %d packets\r\n", numPacketsSearched);

    
    for(int i = 1; i < numPacketTypes + 1; i++){
        if (i == LOG_TEXT || i == LOG_INVALID){ continue; }

        if(packetsFound[i-1] > 0){
            printf("%-13s AVG Data Rate: [%4" PRId32 "ms] : %3" PRIu32 " found\r\n", 
                getPacketName(i), packetDeltaAvg[i-1], packetsFound[i-1]);
        }
        else{
             printf("%-13s AVG Data Rate: [   0ms]\r\n", "SENSOR_IMU");
        }
    }
    printf("======================================\r\n\r\n");
}

int FlashLog::writePacket(uint8_t type, void *packet, ptimer_t pwr_ctr, ptimer_t flight_ctr, FlashLogConfig::State_t state) {
    if (!logInitialized) {
        printf("[FlashLog] ERROR: no init (writePacket)\r\n");
        return FL_ERROR_LOGNOINIT;
    }
    MBED_ASSERT(logInitialized);    //log must have been initialized

    size_t len = getPacketLen(type);

    if(len == 0)
	{
    	// packet type has not been added to getPacketLen()
    	return FL_ERROR_TYPE;
	}

    //Check for bounds
    if(nextPacketAddr < logStart || (nextPacketAddr+MAX_PACKET_LEN) > logEnd)
        return FL_ERROR_BOUNDS;

    //populate the packet's tail
    populatePacketTail(type, len, packet, pwr_ctr, flight_ctr, state);

#ifdef FL_DEBUG
    PRINT_PACKET_BYTES(type, packet);
    printf("Writing packet type %" PRIX8 " to 0x%016" PRIX64 "\r\n", type, nextPacketAddr);
    // printf("Current Tail State: %d\r\n", packet.tail->state);
    printf("Current State: %s\r\n", FlashLogConfig::getStateName(state));
#endif

    int err = writeToLog(packet, nextPacketAddr, len);
    if (!err)
    {
        lastPacketType = type;
        nextPacketAddr += len;
        lastTailAddr = nextPacketAddr - sizeof(struct packet_tail);
        packetsWritten++;
    }
#ifdef FL_DEBUG
    else {
            printf("[FlashLog] error (%d) writing packet type %" PRIX8 " to 0x%016" PRIX64 "\r\n", err, type, nextPacketAddr);
    }
#endif
    return err;
}


int FlashLog::wipeLog(bool complete, Callback<void(float)> progressCallback)
{
    if(!FlashLogConfig::isSPIFlash)
    {    // set scratch buffer to 0xFF to program the log with
        memset(scratchBuffer, 0xFF, blockSize);
    }

    int err=0;
    // hopefully the block device size is always a clean multiple of the sector size
    MBED_ASSERT(blockDev.size() % blockSize == 0);

    // set eraseEndLoc to 1 byte past the last address to erase
    bd_addr_t eraseEndLoc = complete ? logEnd : nextPacketAddr;

    // now erase sectors in order
    printf("Erasing flash log [0x%016" PRIx64 " - 0x%016" PRIx64 ").\r\n", logStart, eraseEndLoc);

    // note: this loop might erase past the last packet location when doing a non-complete erase,
    // that's OK though because we assume everything past there is already erased.
    for(bd_addr_t addr = logStart; addr < eraseEndLoc; addr += eraseBlockSize)
    {
        // Print progress if process has advanced at least 0.01%
        float progress = ((float) addr / static_cast<float>(eraseEndLoc))*100;
		progressCallback(progress);

        if(FlashLogConfig::isSPIFlash)
        {
            err = blockDev.erase(addr, eraseBlockSize);
        }
        else
        {
            err = blockDev.program(scratchBuffer, addr, blockSize);
        }
    }
    printf("[FlashLog] Erase finished!\r\n");

    // reset all FlashLog data
    nextPacketAddr = logStart;
    lastTailAddr = logStart;
    packetsWritten = 0;
    lastPacketType = LOG_INVALID;

    nextPacketToRead = logStart;

	// remove any data that may have been in the write cache
	clearWriteCache();

    cacheBlockAddress = roundDownToNearestBlock(logStart);
    firstWriteToLog = true;
    writeCacheValid = false;
    flashlogTimer.stop();
    flashlogTimer.reset();
    
    return err;
}

FLResultCode FlashLog::binaryDumpIterator(struct log_binary_dump_frame *frame, bool begin)
{
	static bd_addr_t nextReadAddr=logStart; // Address where the next read will start

    //begin a new iteration?
    if (begin)
    {
        nextReadAddr = logStart;
    }
    //check if we're about to read over the end of the log
    if (nextReadAddr >= nextPacketAddr)
    {
        return FL_ITERATION_DONE;
    }

    if(nextReadAddr + sizeof(log_binary_dump_frame) > nextPacketAddr)
	{
    	// must truncate this read, would pass end of log
    	bd_size_t readLen = nextPacketAddr - nextReadAddr;

    	// read partial frame
		readFromLog(frame, nextReadAddr, readLen);

		// fill the rest of the frame with FFs
		memset(&(frame->bytes[readLen]), 0xFF, sizeof(log_binary_dump_frame) - readLen);
	}
    else
	{
		//grab the next frame of data and return
		readFromLog(frame, nextReadAddr, sizeof(log_binary_dump_frame));
	}

    nextReadAddr += sizeof(log_binary_dump_frame);
    return FL_SUCCESS;
}

FLResultCode FlashLog::binaryDumpReverseIterator(struct log_binary_dump_frame *frame, bool begin)
{
	static bd_addr_t nextReadAddrReverse = 0; // one byte after the next byte we want to read

    const int64_t frame_len = sizeof(log_binary_dump_frame);

    // Flag value to indicate that the iterator has reached the log start
    const bd_addr_t endMarker = std::numeric_limits<bd_addr_t>::max();

    //begin a new iteration?
    if (begin)
    {
        nextReadAddrReverse = nextPacketAddr; //address of the byte after the last byte written to the memory
    }
    //check if we're about to read over the end of start of the log
    if (nextReadAddrReverse > logEnd || nextReadAddrReverse == endMarker)
    {
        return FL_ITERATION_DONE;
    }
	else if(nextReadAddrReverse <= logStart + frame_len)
	{
        //not enough data to fill the full frame. will fill it partially
        readFromLog(frame->bytes + frame_len - nextReadAddrReverse, logStart, nextReadAddrReverse);
        memset(&frame->bytes, 0, nextReadAddrReverse); // zero pad start

        // indicate that the packet has reached the end
        nextReadAddrReverse = endMarker;
    }
    else
	{
    	nextReadAddrReverse -= frame_len;

        //grab the next frame of data and return
        readFromLog(frame, nextReadAddrReverse, frame_len);
    }

    //reverse the log_binary_dump_frame
    for(size_t i = 0; i < frame_len/2; i++){
        char temp = frame->bytes[i];
        frame->bytes[i] = frame->bytes[frame_len - i - 1];
        frame->bytes[frame_len - i - 1] = temp;
    }

    return FL_SUCCESS;
}


bd_addr_t FlashLog::getLogCapacity()
{
    return logEnd - logStart;
}

bd_addr_t FlashLog::getLogSize()
{
    return nextPacketAddr - logStart;
}

bd_addr_t FlashLog::getLogStartAddress()
{
    return logStart;
}

bd_addr_t FlashLog::getLastTailAddr()
{
    return lastTailAddr;
}

bd_addr_t FlashLog::findPacketTailBefore(bd_addr_t curPacketTailAddr, struct packet_tail * curPacketTail, bd_addr_t * prevPacketTailAddr, struct packet_tail * prevPacketTail) {
    bd_addr_t searchStartAddress;
    if(tailDescribesValidPacket(curPacketTail, curPacketTailAddr)){
        //the tail they provided is valid, so the type ID is also valid. 
        //compute the previous packet address from this.

        // check to make sure we don't start before the beginning of the log
        if (curPacketTailAddr < logStart + getPacketLen(curPacketTail->typeID))
        {
#ifdef FL_DEBUG
            printf("[findPacketTailBefore] Next Packet will Underflow. NOTAIL");
#endif

            return FL_ERROR_NOTAIL;
        }

        *prevPacketTailAddr = curPacketTailAddr - getPacketLen(curPacketTail->typeID);
        readFromLog(prevPacketTail, *prevPacketTailAddr, sizeof(struct packet_tail));
        

#ifdef FL_DEBUG
        printf("[findPacketTailBefore] Provided Packet is Valid\r\n");
        printPacketTail(curPacketTail);
#endif

        if(tailDescribesValidPacket(prevPacketTail, *prevPacketTailAddr)){
            // the previous packet we found is also valid, so return success
            return FL_SUCCESS;
        }
        else{
            //else continue onto the backtracking from the computed previous packet tail addr
            searchStartAddress = *prevPacketTailAddr;
        }
    }
    else{
        //the tail provided was invalid, so start backtracking at the address of that tail
        searchStartAddress = curPacketTailAddr;
        memcpy(prevPacketTail, curPacketTail, sizeof(struct packet_tail));
#ifdef FL_DEBUG
        printf("[findPacketTailBefore] Provided Packet is Invalid. Copying curPacketTail to prevPacketTail\r\n");
        printPacketTail(curPacketTail);
		printPacketTail(prevPacketTail);
#endif

    }

    bd_addr_t searchEndAddress = 0;
    if(searchStartAddress > MAX_PACKETS_TO_CHECK * MAX_PACKET_LEN)
    {
        searchEndAddress = searchStartAddress - MAX_PACKETS_TO_CHECK * MAX_PACKET_LEN;
    }
    if(searchEndAddress < logStart)
    {
        // handle the case where logStart > 0
        searchEndAddress = logStart;
    }

#ifdef FL_DEBUG
        printf("Searching byte by byte from 0x%" PRIX64 " to 0x%" PRIX64 "\r\n", searchStartAddress, searchEndAddress);
#endif

    // now iterate backwards, byte by byte, until we find a valid packet.
    // We start at the current tail, but we cannot be sure that it is actually a complete,
    // non-INVALID packet, so we may have to go back further.
    for(bd_addr_t tailAddress = searchStartAddress; tailAddress > searchEndAddress; )
    {
#ifdef FL_DEBUG
        printf("Attempting to find packet from possible packet tail at 0x%" PRIX64 "\r\n", tailAddress);
#endif

        if(!tailDescribesValidPacket(prevPacketTail, tailAddress))
        {
            // the packet described by this tail is no good
            // move the current tail one byte backwards in the log
            tailAddress--;
            if(tailAddress > searchEndAddress)
            {
                // the current tail address is still greater than (further into the log) than searchEndAddress
                // so we can read one new byte.
                std::memmove(reinterpret_cast<uint8_t *>(prevPacketTail) + 1, prevPacketTail, sizeof(struct packet_tail) - 1);

                // now read the new byte
                readFromLog(prevPacketTail, tailAddress, 1);
            }
            //else, the loop condition will trigger and exit the for loop
        }
        else 
        {
            // got one!
#ifdef FL_DEBUG
            printf("Found a valid tail at %08" PRIX64 ".\r\n", tailAddress);
            printPacketTail(prevPacketTail);
#endif
            *prevPacketTailAddr = tailAddress;
            return FL_SUCCESS;
        }
    }
    return FL_ERROR_NOTAIL;
}


/* writes the next packet's type to TYPE and fills BUF with the packet's bytes. If
 START=true, start at the first packet in the log. If START==false, continue
 where we left off from the last call returns true if found a good packet, false
 if MAX_READ_FAILURES has been reached with no valid packet

 @param      type   The type
 @param      buf    The buffer
 @param[in]  begin  The start

 @return     { description_of_the_return_value }
*/
FLResultCode FlashLog::packetIterator(uint8_t *type, void *buf, bool begin) {
    int tailOffset = 0;
    bool dataExists = false;
    if (begin == true) {
        nextPacketToRead = logStart;
    }

    if (nextPacketToRead < logStart || nextPacketToRead > logEnd - MAX_PACKET_LEN) {
        //we're going out of the log's bounds.
        return FL_ERROR_BOUNDS;
    }

    //Read a chunk starting at nextPacketToRead, so that we can scan for the tail
    readFromLog(buf, nextPacketToRead, MAX_PACKET_LEN);
#ifdef FL_DEBUG
    PRETTYPRINT_BYTES(buf, MAX_PACKET_LEN, nextPacketToRead);
#endif //FL_DEBUG

    //Check if all 0xFF's
    char *bufc = (char *)buf;
    dataExists = false;
    for (size_t i=0; i<MAX_PACKET_LEN; i++) {
        if (~bufc[i] & 0x000000FF) {
            dataExists = true; // We found a non 0xFF value
            break; // Break out of for loop
        }
    }
    if(!dataExists) {
        printf("ERROR_EMPTY at 0x%016" PRIX64 "\r\n", nextPacketToRead);
        PRETTYPRINT_BYTES(buf, MAX_PACKET_LEN, nextPacketToRead);
        return FL_ERROR_EMPTY; // We read all 0xFF's
    }

    //try to find the tail in the chunk we read
    tailOffset = findTailInBuffer((uint8_t *)buf, MAX_PACKET_LEN);
    if (tailOffset == 0) {
        //there wasn't a valid packet/tail in that segment! increment nextPacketToRead to the next section of MAX_PACKET_LEN bytes,
        //but go back by a few bytes (2 bytes for a 3 byte magic code) in case our last read terminated in the middle of a magic code!
        /*     This PROBABLY means the magic constant was corrupted, which means we're going to lose this packet. But we'll find the next one.
                We need to note this possibility when reviewing the main FSM, however--even though writes to this logfile are essentially atomic
                because of all the validity checks, that doesn't mean that the last packet write has a chance to finish. 
                Therefore, there can NOT be any state transition conditions that can't be guaranteed to last the length of a brownout, 
                i.e. `flight_timer >= CONST` is fine, while `flight_timer == CONST` is not. */
        nextPacketToRead += (MAX_PACKET_LEN-2);
        return FL_ERROR_NOTAIL;
    }

    //TODO consider eliminating extraneous SPI reads by having only the relevant errors re-read here
    /*     in other words, if the last packet was nominal, and this one is too, then we know the packet is in
            the original read of MAX_PACKET_LEN and we don't have to read again to get it. just look in buf.
            This either means keeping an internal class variable (kinda silly imo), OR performing iteration
            through bad packets within this function. This has the questionably positive effect of giving the user
            of the API less control over what they want to print for bad packets. */
    readFromLog(buf, nextPacketToRead+tailOffset, sizeof(struct packet_tail));
    struct packet_tail *pt = (struct packet_tail *)buf;
    //use the tail to find the type of the packet, and ensure the type gives a valid length
    *type = pt->typeID;
    checksum_t tmpChecksum = pt->checksum;
    pt = NULL; //do not access the pt pointer after this because buf is about to be used to store the packet itself
    size_t len = getPacketLen(*type);
    if (len == 0) {
        //the packet type is invalid, and this packet is likely corrupted. Try the next one.
        nextPacketToRead += tailOffset+offsetof(struct packet_tail, state);
        return FL_ERROR_TYPE;
    }
    //Read the packet contents based on the type size, check left bound first
    bd_addr_t packetAddr = nextPacketToRead+tailOffset+sizeof(struct packet_tail)-len;
    if(packetAddr < logStart) {
        //packet start as detected here is somehow before the start of the log.
        return FL_ERROR_BOUNDS;
    }

    //TODO see above
    readFromLog(buf, packetAddr, len);
    //Validate checksum
    if(calculateChecksum(len,buf) != tmpChecksum){
        //Either our packet got corrupted, or we found a fake magic constant in our data.
        //Can't easily distinguish between the two as far as I (Ian) can tell,
        //so start reading next attempt at the end of this magic constant.
        nextPacketToRead += tailOffset+offsetof(struct packet_tail, state);
        return FL_ERROR_CHECKSUM;
    }
    //Valid packet, so type and buf now hold valid contents and the calling task can process it.
    //Set the next packet pointer to start looking after this tail for the next time this function is called
    nextPacketToRead += tailOffset+sizeof(struct packet_tail);
    return FL_SUCCESS;
}

/* Reads the most recent packet from flash and copies it to BUF. That packet is of
 type TYPE. This is just useful for debugging purposes, not for flight code. It
 doesn't check for brownouts/ that we've initialized the log. Also, to use it
 effectively you have to know what type of packet should have been written last.

 @param[in]  type  The type
 @param      buf   The buffer

 @return     { description_of_the_return_value }
*/
FLResultCode FlashLog::readLastPacket (uint8_t type, void *buf) {
    bd_addr_t lastPacketAddr = nextPacketAddr - getPacketLen(type);
#ifdef FL_DEBUG
  printf("reading from last packet, type %" PRIX8 ": %" PRIX64 "\n\r", type, lastTailAddr);
    #endif //FL_DEBUG
    return readFromLog(buf, lastPacketAddr, getPacketLen(type));
}

/* Debugging function that prints NBYTESPRE bytes before nextPacketAddr, and
 NBYTESPOST bytes after.

 @param[in]  nBytesPre   The bytes pre
 @param[in]  nBytesPost  The bytes post
*/
void FlashLog::printLastNBytes (size_t nBytesPre, size_t nBytesPost) 
{
    uint8_t byte;
    bd_addr_t startaddr = (nextPacketAddr - nBytesPre);
    for (bd_addr_t i=startaddr >= logStart ? startaddr : logStart; i<nextPacketAddr; i++) {
        readFromLog(&byte, i, 1);
      if (i==lastTailAddr) printf("\r\n[lastTailAddr]\r\n");
        if ((i-nextPacketAddr)%16 == 0) printf("\r\n[%016" PRIX64 "]: ", i);
      else if ((i-nextPacketAddr)%4 == 0) printf("-");
      printf("%02x", byte);
    }
    printf("\r\n[nextPacketAddr]\r\n");
    for (bd_addr_t i=nextPacketAddr; i<nextPacketAddr+nBytesPost; i++) {
        readFromLog(&byte, i, 1);
        if ((i-nextPacketAddr)%16 == 0) printf("\r\n[%016" PRIX64 "]: ", i);
      else if ((i-nextPacketAddr)%4 == 0) printf("-");
      printf("%02x", byte);
    }
    printf("\r\n");
}

/* returns a pointer to the enclosing packet given its tail struct
 *
 * @param      tail  The tail
 *
 * @return     packet pointer
 */
void *FlashLog::tailToPacket(struct packet_tail* tail) {
    size_t len = getPacketLen(tail->typeID);
    return (void*) ((ptrdiff_t) tail - len + sizeof(struct packet_tail));
}

/* Populate a packet_tail with correct values from the timers, fsm state, etc
 *
 * @param[in]  type    The type
 * @param[in]  len     The length
 * @param      packet  The packet
 */
void FlashLog::populatePacketTail(uint8_t type, size_t len, void *packet, ptimer_t pwr_ctr, ptimer_t flight_ctr, FlashLogConfig::State_t state) {
    //get a pointer to the packet's tail
    struct packet_tail *tail = (struct packet_tail*) (((ptrdiff_t) packet + len) - (int) sizeof(struct packet_tail));
    //populate the tail
    tail->magic1 = LOG_PACKET_MAGIC1;
    tail->magic2 = LOG_PACKET_MAGIC2;
    tail->magic3 = LOG_PACKET_MAGIC3;
    tail->typeID = type;
    //we only need to fill in values for the rest of the packet if it's a valid packet
    if (type == LOG_INVALID) return;
    else {
        tail->state = static_cast<uint8_t>(state);
        tail->pwr_ctr = pwr_ctr;
        tail->flight_ctr = flight_ctr;
        tail->checksum = calculateChecksum(len, packet);
    }
}

/* Calculates the crc32 checksum of PACKET given its total length LEN in bytes.
 Because every packet ends in packet_tail, this function can ensure that we
 don't use the magic and checksum bits at the end of the tail in the checksum
 calculation.

 @param[in]  len     The length of the packet
 @param      packet  The packet

 @return     The checksum.
*/
checksum_t FlashLog::calculateChecksum(size_t len, void *packet) {
    checksum_t crc = 0;
    // We calculate the checksum based on all the data+tail bytes before it.
    size_t s = len-sizeof(struct packet_tail)+offsetof(struct packet_tail, checksum);
    crc32(packet, s, &crc);
    return crc;
}

/* crc32 functions from http://home.thep.lu.se/~bjorn/crc/
 *
 * Simple public domain implementation of the standard CRC32 checksum.
 */

/* Helper function for generating crc32 factor table

@param[in]  r     table index

@return     table value
*/
uint32_t crc32_for_byte(uint32_t r) {
  for(int j = 0; j < 8; ++j)
    r = (r & 1? 0: (uint32_t)0xedb88320l) ^ r >> 1;
  return r ^ (uint32_t)0xff000000l;
}

/* Calculates the crc checksum of DATA, which has length N_BYTES, storing it in the
uint32_t pointed to by CRC.

@param[in]  data     The data
@param[in]  n_bytes  The length of the data in bytes
@param      crc      The checksum output
*/
void crc32(const void *data, size_t n_bytes, uint32_t* crc)
{
  *crc = 0;
  static uint32_t table[0x100];
  if(!*table)
    for(size_t i = 0; i < 0x100; ++i)
      table[i] = crc32_for_byte(i);
  for(size_t i = 0; i < n_bytes; ++i)
    *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}

/* takes in a buffer BUF of size LEN and finds the start of the first tail, if
 any, contained within it. If REVERSE==true, then find the start of the LAST
 tail contained in BUF.

 Note that this just indicates where a valid packet _might_ be, a >=0 return value
 is not proof positive that there is a valid packet.

 Returns the tail's distance in bytes from the start of BUF, or -1 if the tail
 is not found

 @param      buf      The buffer
 @param[in]  len      The length
 @param[in]  reverse  The reverse

 @return     { description_of_the_return_value }
*/
int FlashLog::findTailInBuffer(uint8_t *buf, size_t len, bool reverse) {
    uint16_t mag1_candidate;
    uint8_t mag2_candidate;
    int i=0, i_end=len-3, inc=1;
    if (reverse) {
        i=len-3;
        i_end = 0;
        inc = -1;
    }
    for (; i!=i_end; i+=inc) {
        // printf("findTailInBuffer() i=%d, i_end=%d, inc=%d\r\n", i, i_end, inc);
      mag1_candidate = (buf[i+1] << 8) | buf[i];   //accounts for endianness disagreement
      mag2_candidate = buf[i+2];
      if (mag1_candidate == LOG_PACKET_MAGIC1 && mag2_candidate == LOG_PACKET_MAGIC2) {
          //we've got the tail
          return i;
      }
    }
#ifdef FL_DEBUG
    printf("Tail not found in buffer:\r\n");
    PRINT_BYTES(buf, len);
#endif
    return -1;
}

void FlashLog::clearWriteCache()
{
	memset(writeCache, 0xFF, blockSize);
	writeCacheValid = false;
}

/* Uses the magic fields to confirm that the contents of buf are actually a
 * packet_tail
 *
 * @param      buf   The buffer containing the tail
 *
 * @return     True if tail, False otherwise.
 */
bool isTail(void *buf) {
    struct packet_tail *tailp = (struct packet_tail *) buf;
    return tailp->magic1 == LOG_PACKET_MAGIC1 && tailp->magic2 == LOG_PACKET_MAGIC2 && tailp->magic3 == LOG_PACKET_MAGIC3;
}
