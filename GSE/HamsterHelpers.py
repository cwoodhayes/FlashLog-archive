# Miscellaneous functions that help with HAMSTER
import re, sys, os

##
## @brief      seeks through file until it finds a dump open tag
##
## @param      ifile  The file to seek through. Must already be opened
##
## @return     true if bindump found, false if not
##
def seekToDump(ifile, dump_open_tag):
	# find the binary dump tag in the file
	ifile.seek(0)
	filestring = ifile.read()
	bindump_tag_bytes = dump_open_tag.encode()
	offset = filestring.find(bindump_tag_bytes)
	if offset == -1:
		return False
	# skip to the end of the tag
	offset = offset + len(bindump_tag_bytes)
	# skip over any combo of \r and \n after the dump tag
	for i in [0,1]:
		if (filestring[offset] == ord('\r')) or (filestring[offset] == ord('\n')):
			offset = offset + 1
		else:
			break
	#seek to this spot in the file
	ifile.seek(offset)
	return True

###### Random helper functions that I copypasta around all the time

##
## @brief      returns an unused filename by appending an unused ordinal number to "filename" 
## 
##
## @param      filename   The intended filename of the file
## @param      extension  The desired file extension
##
## @return     the available filename 
## 
## @author 	   Conor Hayes
##
def first_unused_filename(filename):
	#separate out the filename and extension, handling the case of 
	#files with periods in them
	namelist = filename.split('.')
	extension = ''
	if len(namelist) > 1:
		#there is an extension
		extension += '.'+namelist[-1]
		if len(namelist) > 2:
			filename = '.'.join(namelist[0:-1])
		else:
			filename = namelist[0]

	r_ext = re.escape(extension)
	filename += extension
	if os.path.exists(filename):
		filename =  re.sub(r_ext+r'$', r'1{}'.format(extension), filename)
	ctr = 2
	while (os.path.exists(filename)):
		filename = re.sub(r'\d+(?={ext})'.format(ext=r_ext), str(ctr), filename)
		ctr += 1
	return filename
