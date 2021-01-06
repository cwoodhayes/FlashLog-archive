#!/usr/local/bin/python3
# Author: Conor Hayes
import re, struct, argparse, csv
import sys, os
import HamsterPacket
import HamsterHelpers
import pprint
import statistics
import matplotlib.pyplot as plt
import numpy as np
from peak_detection import get_persistent_homology
from prettytable import PrettyTable


##
## @brief      populate a HamsterPackets object with the contents of a dump
##
## @param      ifile    The input file, pre-seeked to the beginning of a dump
## @param      packets  The HamsterPackets object to populate
##
## @return     0 if no error
##
def parsePackets(ifile, packets):
	total_bytes_read = 0
	tail_search_fifo = bytearray(ifile.read(HamsterPacket.LEN_TAIL-1))
	total_bytes_read += len(tail_search_fifo)
	# tail_search_fifo = bytearray(ifile.read(HamsterPacket.packet_len[1])) #DEBUG
	# print(tail_search_fifo)
	# tail_search_fifo = bytearray(ifile.read(HamsterPacket.LEN_TAIL))
	# print(tail_search_fifo)
	# return 0

	bytes_discarded = 0

	next_byte = ifile.read(1)
	#iterate through the entire input file, catching packet tails where possible
	while next_byte:
		tail_search_fifo.append(next_byte[0])
		total_bytes_read += 1
		possible_tail = struct.unpack(HamsterPacket.FORMAT_TAIL, tail_search_fifo[-HamsterPacket.LEN_TAIL:])

		# if the fifo now contains a tail
		if (possible_tail[0] == HamsterPacket.MAGIC1 and possible_tail[1] == HamsterPacket.MAGIC2 and possible_tail[-1] == HamsterPacket.MAGIC3):
			#categorize and save it, then flush the fifo
			packet_len = packets.addPacket(tail_search_fifo, possible_tail)
			cur_bytes_discarded = len(tail_search_fifo) - (packet_len if packet_len else 0)
			if cur_bytes_discarded > 0:
				print(cur_bytes_discarded, "bytes discared. Packet #", packets.packetNumber)
			bytes_discarded += cur_bytes_discarded
			tail_search_fifo = bytearray(ifile.read(HamsterPacket.LEN_TAIL-1))
			total_bytes_read += len(tail_search_fifo)

		# otherwise, grab another byte and keep going
		next_byte = ifile.read(1)

		#quit if we're not getting valid data.
		if (len(tail_search_fifo) > HamsterPacket.MAX_ERROR_PACKET_LEN and not HamsterPacket.IGNORE_MAX_ERROR):
			print('Error: exceeded max packet length in search of tail (offset 0x{:X}).'.format(ifile.tell()))
			#print(tail_search_fifo)
			#return 1

	print(bytes_discarded, '/', total_bytes_read, 'bytes discarded in total due to errors or initial padding.')
	return 0

##
## @brief      Export HamsterPackets to nice CSV files, one file per packet type, in a new directory
##
## @param      dirname  The output directory name
## @param      packets     The HamsterPackets object to export
##
## @return     { description_of_the_return_value }
##
def exportPackets(dirname, packets):
	os.mkdir(dirname)
	for typeID in packets:
		with open("{}/{}.csv".format(dirname, HamsterPacket.packet_name[typeID]) , 'w', newline='') as ofile:
			csvwriter = csv.writer(ofile)
			header = ['Packet #', 'Checksum Status', 'Output Checksum'] + HamsterPacket.tail_fields \
							 + HamsterPacket.packet_fields[typeID] + ['Packet (hex)']
			csvwriter.writerow(header)
			for line in packets[typeID]:
				csvwriter.writerow(line)

	# create the top-level tails-only CSV file
	with open("{}/tails.csv".format(dirname) , 'w', newline='') as ofile:
		csvwriter = csv.writer(ofile)
		header = ['Packet #', 'Packet Type', 'Checksum Status', 'Output Checksum'] + HamsterPacket.tail_fields
		csvwriter.writerow(header)
		for line in packets.tails:
			csvwriter.writerow(line)

##
## @brief      Populate the argparse parser with its argument format
##
## @param      parser  pre-instatiated argparse parser
##
## @return     the populated parser
##
def addargs(parser):
	parser.add_argument('input_file', metavar='FILE')
	parser.add_argument('-o', '--output-dir', metavar='DIRECTORY',
		help='This program generates several CSV files. Use this option to specify\
			  the name of the directory we place them in.')
	return parser

def padArray(arr, pad):
	if(not hasattr(arr, "__len__")):
		return arr

	myStr =  "["
	for idx,value in enumerate(arr):
		x = format(value, str(pad))
		myStr  = myStr + x
		if(idx < len(arr)-1):
			myStr  = myStr + ", "

	myStr = myStr + "]"
	return myStr

def makeTable(title, comment, report, index, padTo):

	table = PrettyTable()
	table.field_names = ["State", "ACCEL", "BARO", "GPS", "BNO", "POWER", "TEMP", "ADIS"]
	counter = 1
	for state, statePacketTypes in report.items():
		row = [None] * len(table.field_names)
		row[0] = state
		counter = 1
		for sensorName in table.field_names[1:]:
			row[counter] = padArray(statePacketTypes[sensorName][index], padTo)  if sensorName in statePacketTypes else None
			counter = counter + 1

		table.add_row(row)

	print(title)
	print(comment)
	print("============")
	print(table)
	print("\n")


#For each state, what is the packet type frequency
def statePacketFrequency (packets):
	deltas = {}
	report = {}

	prevState = None
	count = 0
	pp = pprint.PrettyPrinter(indent=4)

	prevTails = {};
	prevProcessedTail = None

	TAIL_STATE   = 6
	TAIL_TYPE    = 1
	TAIL_CHECKSUM_STATUS = 2
	TAIL_PWR_CTR = 10
	mSEC_TO_uSEC  = 1000

	for tail in packets.tails:
		#6th element in tail is the state
		# print (tail[6], prevState)
		#1st element is packet type
		
		curTailState = HamsterPacket.packet_state[tail[TAIL_STATE]]
		curTailType = str(tail[TAIL_TYPE])
		

		if tail[TAIL_CHECKSUM_STATUS] != 'OK':
			continue

		if curTailState not in deltas:
			deltas[curTailState] = {}
			deltas[curTailState][curTailType] = []
		elif curTailType not in deltas[curTailState]:
			deltas[curTailState][curTailType] = []
			
		if (prevProcessedTail != None) and (tail[TAIL_STATE] != prevProcessedTail[TAIL_STATE]):
			#we just had a transition, so all "prevTails" are invalid
			for tailType in prevTails.keys():
				prevTails[tailType] = None


		if curTailType not in prevTails:
			# no point processing this packet, its the first of this type
			prevTails[curTailType] = tail
		elif prevTails[curTailType] == None:
			# no point processing this packet, its the first of this type after a transition
			prevTails[curTailType] = tail
		else:
			#find the delta from this packet and the previous packet of this type
			delta = (int(tail[TAIL_PWR_CTR]) - int(prevTails[curTailType][TAIL_PWR_CTR])) / mSEC_TO_uSEC
			deltas[curTailState][curTailType].append(delta)
			prevTails[curTailType] = tail

		prevProcessedTail = tail

	largestBinEdge = 0
	largestPersistence = 0

	for state, statePacketTypes in deltas.items():
		for packetType, statePacketDeltas in statePacketTypes.items():
			if(state not in report):
				report[state] = {}

			if(len(statePacketDeltas) < 2):
				continue
			else:
				binwidth = 2
				bins = bins=range(int(min(statePacketDeltas)), int(max(statePacketDeltas)) + binwidth, binwidth)
				hist, bin_edges = np.histogram(statePacketDeltas, bins=bins)
				peaks = get_persistent_homology(hist)
				periodPeaks = []
				persistence = []
				for i in range(0,3):
					if(i < len(peaks)):
						periodPeaks.append(bin_edges[peaks[i].born])
						p = peaks[i].get_persistence(hist)
						persistence.append(p)
						if(peaks[i].born > largestBinEdge):
							largestBinEdge = peaks[i].born
						if(p > largestPersistence):
							largestPersistence = p

				report[state][packetType] = {
					"Average Period" : statistics.mean(statePacketDeltas),
					"Standard Deviation Period" : statistics.pstdev(statePacketDeltas),
					"Period Peaks" : periodPeaks,
					"Persistence" : persistence
				}
	print("\n\nPACKET FREQUENCY REPORT")
	print("=======================")
	
	makeTable("Averages", "", report, "Average Period", 0)
	makeTable("Peaks"   ,
"""
Each cell represents the three most significant peaks in the distribution of packet seperation periods
Units are in milliseconds
""", report, "Period Peaks", len(str(largestBinEdge)))

	makeTable("Persistence", "", report, "Persistence", len(str(largestPersistence)))

######### MAIN

def main():
	parser = addargs(argparse.ArgumentParser(description='Parser for HAMSTER\'s FlashLog data. Generates 1 CSV file per packet type.'))
	args = parser.parse_args()
	if not args.output_dir:
		args.output_dir = args.input_file.split('.')[0] + '-csv'
		args.output_dir = HamsterHelpers.first_unused_filename(args.output_dir)

	packets = HamsterPacket.HamsterPackets()
	with open(args.input_file, 'rb') as ifile:
		if not HamsterHelpers.seekToDump(ifile, HamsterPacket.BINDUMP_OPEN_TAG):
			print('No dump found in file "{}"'.format(args.input_file))
			return 1
		else:
			print("Dump found. Parsing...")
			try:
				parsePackets(ifile, packets)
			except struct.error:
				print('Stopped parsing at ox{:08x} due to parse error.'.format(ifile.tell()))

	print('Dump contents extracted.')
	packets.reportContents()
	print('Exporting to "{}"'.format(args.output_dir))
	exportPackets(args.output_dir, packets)
	print('Dump contents successfully exported to "{}"'.format(args.output_dir))

	statePacketFrequency (packets)
	return 0

if __name__ == '__main__':
	sys.exit(main())
