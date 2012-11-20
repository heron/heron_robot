#!/usr/bin/python
#
# Receives UDP multicast datagrams as broadcast by a Microhard radio
# in UDP Point to Multipoint(P) mode, and forwards them to a selected
# serial port, optionally also printing them to stdout.
#

import sys
import serial
from optparse import OptionParser

try:
  import multicast
except ImportError:
  print "Script requires py-multicast. Please install:"
  print "  sudo apt-get install pip"
  print "  sudo pip install py-multicast"
  exit(1)

parser = OptionParser(usage="usage: %prog [/dev/tty] [options]")
parser.add_option("-g", "--group", dest="group",
                  help="multicast IP", metavar="IP", default="224.1.1.1")
parser.add_option("-p", "--port", dest="port",
                  help="multicast port", metavar="PORT", default=20001)
parser.add_option("-d", "--device", dest="device",
                  help="multicast device", metavar="ETH", default="eth0")
parser.add_option("-b", "--baud", dest="baud",
                  help="serial baud rate", metavar="RATE", default=115200)
parser.add_option("-v", "--verbose", dest="verbose", action="store_true",
                  help="echo received messages to stdout")

(options, args) = parser.parse_args()

receiver = multicast.MulticastUDPReceiver(options.device, options.group, options.port)

ser = None
if len(args) == 1:
  ser = serial.Serial(port=args[0], baudrate=options.baud, timeout=0)
  ser.open()

try:
  while True:
    s = receiver.read(10240)
    if ser:
      ser.write(s)
      ser.flush()
    if options.verbose:
      sys.stdout.write(str(s).encode("string_escape"))
      sys.stdout.flush()

except:
  if ser:
    ser.close()
  raise
