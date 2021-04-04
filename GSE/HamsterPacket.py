#!/usr/local/bin/python3
# Author: Conor Hayes
import re, struct, argparse, os, sys, binascii

########## Various constants

# packet information, indexed by each packet type's typeID
packet_name = [
    "__error__",
    "TEXT",
    "TEMP",
    "ACCEL",
    "BNO",
    "GPS",
    "BARO",
    "INVALID",
    "POWER",
    "ADIS",
    "RANGEFINDER",
]

# packet state list
packet_state = [
    "INIT",
    "FLY",
    "STANDBY",
    "DEBUG",
    "WIPE",
    "TERMINAL_COUNT",
    "ARMED",
    "CHECK_DISK",
    "LANDED",
    "ASCENT",
    "DESCENT",
    "DROGUES_FIRING",
    "DROGUES_FIRED",
    "DUMP",
    "NONE",
    "END_ENUM_STATE",
]

BINDUMP_OPEN_TAG = "<bindump>"
BINDUMP_CLOSE_TAG = "</bindump>"
HEXDUMP_OPEN_TAG = "<hexdump>"
HEXDUMP_CLOSE_TAG = "</hexdump>"
HEXDUMP_REV_OPEN_TAG = "<hexdump-reverse>"
HEXDUMP_REV_CLOSE_TAG = "</hexdump-reverse>"
BASE64_REV_OPEN_TAG = "<base64-reverse>"
BASE64_REV_CLOSE_TAG = "</base64-reverse>"
COMPRESSED_HEX_REV_OPEN_TAG = "<compressed-hex-reverse>"
COMPRESSED_HEX_REV_CLOSE_TAG = "</compressed-hex-reverse>"
COMPRESSED_BASE64_REV_OPEN_TAG = "<compressed-base64-reverse>"
COMPRESSED_BASE64_REV_CLOSE_TAG = "</compressed-base64-reverse>"

HEX_DUMP_FRAME_PRINT_TERMINATOR = (
    "\r\n"  # what's printed out after we print a dump frame
)

# TODO not sure about endianness, so just try one and if
# it doesn't work then switch it around
packet_format = [  # See Packet.h for these struct declarations
    r"",  # this is an error. no real packet has typeID 0
    r"<64s",  # text
    r"<2f",  # temp
    r"<4h",  # accel
    r"<4f3f3f2c2Bi",  # bno
    r"<2d4iIH10B",  # gps
    r"<fifi",  # baro
    r"",  # invalid
    r"<f4Bff",  # power
    r"<7fH2B",  # adis
    r'<dfB3x', #rangefinder
]


packet_len = [struct.calcsize(FORMAT) for FORMAT in packet_format]

# tail format & contents. See Packet.h if you're confused
FORMAT_TAIL = r"<HBBBBHQQII"
LEN_TAIL = struct.calcsize(FORMAT_TAIL)
REVERSE_CHECKSUM_OFFSET = (
    8  # offset of the start of the checksum field fron the end of the packet tail
)
MAGIC1 = int(b"0xAAAA", 16)
MAGIC2 = int(b"0xAA", 16)
MAGIC3 = int(b"0xCCCCCCCC", 16)
tail_fields = [
    "magic1",
    "magic2",
    "state",
    "typeID",
    "PAD",
    "PAD",
    "pwr_ctr",
    "flight_ctr",
    "checksum",
    "magic3",
]

error_fields = [
    "err_data",
]

text_fields = ["msg"]
temp_fields = ["temp", "PAD"]
accel_fields = ["ax", "ay", "az", "PAD"]
bno_fields = [
    "quatX",
    "quatY",
    "quatZ",
    "quatW",

    "ax",
    "ay",
    "az",
    "magX",

    "magY",
    "magZ",
    "calib",
    "stability",

    "PAD",
    "PAD"
]
gps_fields = [
    "latitude",
    "longitude",
    "height", "northVel",
    "eastVel","downVel",
    "posAccuracy","year", "month", "day",
    "hour", "minute", "second", "numSatellites", "fixQuality", "PAD", "PAD", "PAD"
]

baro_fields = ["altitude", "pressure", "temperature", "PAD"]
invalid_fields = ["err_data"]

power_fields = [
    "battVoltage",
    "chargePercent", "PAD", "PAD", "PAD",
    "battCurrent",
    "reg5VCurrent",
]

adis_fields = [
    "gyrox", "gyroy",
    "gyroz", "ax",
    "ay", "az",
    "temp", "datacnt", "PAD", "PAD",
]

rangefinder_fields = [
	"distance",
	"rssi",
	"linkQual",
]

packet_fields = [
    error_fields,
    text_fields,
    temp_fields,
    accel_fields,
    bno_fields,
    gps_fields,
    baro_fields,
    invalid_fields,
    power_fields,
    adis_fields,
    rangefinder_fields,
]

# other constants
MAX_PACKET_LEN = max(packet_len) + LEN_TAIL  # text is the longest packet type
MAX_ERROR_PACKET_LEN = (
    MAX_PACKET_LEN + LEN_TAIL - 1
)  # text packet written except 1 byte, then LOG_INVALID tail tacked on
IGNORE_MAX_ERROR = True
##
## @brief      Class for containing HAMSTER's FlashLog packets
##
class HamsterPackets(dict):
    def __init__(self):
        self.packetNumber = 0
        # initialize the lists for each type of packet
        for typeID in range(0, len(packet_name)):
            self[typeID] = list()

        # initialize the tails list, which stores an ordered list of all packets
        self.tails = []

    ##
    ## @brief      Adds a packet to the container
    ##
    ## @param      self  The object
    ## @param      data  Packet data as a bytearray() object
    ## @param      tail  Packet tail as a list, returned by struct.unpack
    ##
    ## @return     Length of packet used in bytes
    ##
    def addPacket(self, packet_bytes, tail_struct):
        packet_data_bytes = packet_bytes[:-LEN_TAIL]
        packet_type = tail_struct[3]
        data_len = LEN_TAIL
        if packet_type == 0:
            raise Exception("Invalid packet type 0.")
        if (packet_type == 7) or (packet_type == 0):  # invalid packet handling
            data_struct = [packet_data_bytes.hex()]
            checksum_csv_cells = ["INVALID", "N/A"]
        else:  # valid packet handling
            try:
                cur_packet_bytes = packet_data_bytes[-packet_len[packet_type] :]
                data_struct = struct.unpack(
                    packet_format[packet_type], cur_packet_bytes
                )
                data_len += len(cur_packet_bytes)
            except Exception as e:
                print("Data could not be unpacked. Packet #" + str(self.packetNumber))
                # print('Packet Type: ' + str(packet_type))
                # print('ERROR: '  + str(e))
                # print('\tdata: {}'.format(cur_packet_bytes.hex()))
                # print('\ttail: {}'.format(tail_struc))

                packet_csv_cells = [self.packetNumber]
                checksum_csv_cells = ["N/A", "N/A"]
                data_struct = tuple()
                packet_type = 0  # invalid packet

                # TODO make this not cancer
                packet_csv_cells.extend(checksum_csv_cells)
                packet_csv_cells.extend(tail_struct)
                packet_csv_cells.extend(data_struct)
                packet_csv_cells.append(packet_bytes.hex())
                self[packet_type].append(packet_csv_cells)

                # create a CSV row for this packet in the tails spreadsheet
                tail_csv_cells = [self.packetNumber, packet_name[packet_type]]
                tail_csv_cells.extend(checksum_csv_cells)
                tail_csv_cells.extend(tail_struct)
                self.tails.append(tail_csv_cells)
                self.packetNumber = self.packetNumber + 1
                return

            # validate the packet checksum, adding 2 columns to the packet and tail csv's
            data_to_checksum = packet_bytes[
                -packet_len[packet_type] - LEN_TAIL : -REVERSE_CHECKSUM_OFFSET
            ]
            checksum = binascii.crc32(data_to_checksum)

            checksum_csv_cells = []
            if checksum == tail_struct[8]:  # checksum is valid
                checksum_csv_cells.append("OK")
            else:
                checksum_csv_cells.append("INVALID")
            checksum_csv_cells.append(checksum)

            # special packet-type based processing
            if packet_type == 1:  # make the text packet messages look nice
                try:
                    end_idx = data_struct[0].find(b"\x00")
                    data_struct = list(data_struct)
                    data_struct[0] = data_struct[0][:end_idx].decode("ascii")
                except UnicodeDecodeError as e:
                    print("Could not decode text packet {}".format(data_struct[0]))

        # create a CSV row for this packet in its spreadsheet
        packet_csv_cells = [self.packetNumber]
        packet_csv_cells.extend(checksum_csv_cells)
        packet_csv_cells.extend(tail_struct)
        packet_csv_cells.extend(data_struct)
        packet_csv_cells.append(packet_bytes.hex())
        self[packet_type].append(packet_csv_cells)

        # create a CSV row for this packet in the tails spreadsheet
        tail_csv_cells = [self.packetNumber, packet_name[packet_type]]
        tail_csv_cells.extend(checksum_csv_cells)
        tail_csv_cells.extend(tail_struct)
        self.tails.append(tail_csv_cells)
        self.packetNumber = self.packetNumber + 1

        return data_len

    ##
    ## @brief      Neatly prints the contents of the data structure
    ##
    ## @return     0
    ##
    def reportContents(self):
        total_packets = 0
        for i in range(0, len(packet_name)):
            invalid_packets = 0
            for csv_cells in self[i]:
                invalid_packets += 1 if csv_cells[1] != "OK" else 0
            print(
                "\t{: <20}: {} ({} invalid) packets".format(
                    packet_name[i], len(self[i]), invalid_packets
                )
            )
            total_packets = total_packets + len(self[i])
        print("\t{:_<40}".format(""))
        print("\t{: <20}: {}".format("Total # of packets", total_packets))


# print out the binary of a packet tail for debugging
def print_tail(unpacked_struct_tail):
    print(
        "[{:04X}-{:02X}]".format(unpacked_struct_tail[0], unpacked_struct_tail[1]),
        end="",
    )
    for field in unpacked_struct_tail[2:-1]:
        print("{0:X}-".format(field), end="")
    print("[{:08X}]".format(unpacked_struct_tail[-1]))
