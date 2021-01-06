# GSE (Ground Support Equipment)
Folder for software intended to support HAMSTER or our flight avionics,
but which doesn't run on the unit itself. 

## Directory Contents
|Item													|Description|
|-----------------------------|-----------------------------|
| FlashLogParser.py 	| Parser for binary dumps from the HAMSTER's onboard memory. |
| GetHamsterLog.py 		| Automagically grabs the contents of HAMSTER's flash memory and saves it in a binary file on your hard drive. |
| HamsterPacket.py 		| Various helper functions for FlashLogParser.py |

## Extracting Data from Hamster
The contents of this directory allow the user to easily extract the
data from the HAMSTER's flash memory chip, parse it, and save it in
a neatly organized CSV format for examination or further processing.
To do this, follow the steps below:

1. Flash the FlashLog Test Suite to the HAMSTER by executing
`make flash-test_flash` from your FSW build directory. (If you don't know
how to do this, read through this repo's top-level README.) Wait a few
seconds for the programming to complete.
2. Check the serial baud rate of the HAMSTER by looking in
`FSW/hamster-core/src/SerialPort.cpp` for the `BAUDRATE` macro.
3. Run `./GetHamsterLog.py -b BAUDRATE PORT`, where `BAUDRATE` and `PORT`
are the serial baud rate of the HAMSTER software and the MBED's assigned
device file, respectively. This will transfer the contents of HAMSTER's
memory to your hard drive in the form of the file 'hamster.log'
4. Run `./FlashLogParser hamster.log`. This will generate the directory
`hamster-csv` and a number of CSV files below it corresponding to the
different types of packets in the HAMSTER's memory.