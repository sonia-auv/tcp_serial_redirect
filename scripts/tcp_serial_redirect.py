#!/usr/bin/env python

# (C) 2002-2006 Chris Liechti <cliechti@gmx.net>
# redirect data from a TCP/IP connection to a serial port and vice versa
# requires Python 2.2 'cause socket.sendall is used

# Modified version by Tennessee Carmel-Veilleux (May 9th 2008)
# Adds:
#    --file and --freq option for serial port playback from file to emulate
#    serial devices.
#
#    --sniff to log the input from the serial port to stdout

import os
import serial
import socket
import sys
import threading
import time
from optparse import (OptionParser)

import rospy

try:
    True
except NameError:
    True = 1
    False = 0


class TCPSerialRedirector:
    def __init__(self, serial, socket, filename, freq, sniff):
        self.serial = serial
        self.socket = socket
        self.sniff = sniff
        if filename:
            self.fin = file(filename, "r")
            self.sleeptime = 1.0 / freq
        else:
            self.fin = None
        self.count_read_data = 0
        self.max_read_data_print = 200
        self.count_write_data = 0
        self.max_write_data_print = 200

    def shortcut(self):
        """connect the serial port to the tcp port by copying everything
           from one side to the other"""
        self.alive = True
        self.thread_read = threading.Thread(target=self.reader)
        self.thread_read.setDaemon(1)
        self.thread_read.start()
        self.writer()

    def reader(self):
        """loop forever and copy serial->socket"""
        while rospy.is_shutdown():
            try:
                if self.fin:
                    data = self.fin.readline()  # Read a line from the file instead of from the serial port
                    if data == "":
                        self.fin.seek(0)
                    else:
                        # Add CR+LF at the end, no matter what line ending was present
                        data = data.rstrip() + "\r\n"
                else:
                    data = self.serial.read(1)  # read one, blocking
                    n = self.serial.inWaiting()  # look if there is more
                    if n:
                        data = data + self.serial.read(
                            n)  # and get as much as possible

                if data:
                    # Pipe to stdout if sniffing
                    if self.sniff:
                        print(data)
                        sys.stdout.flush()

                    self.count_read_data += 1
                    if not (self.count_read_data % self.max_read_data_print):
                        print("read %d data." % self.count_read_data)
                        sys.stdout.flush()
                    if self.count_read_data >= sys.maxint - 2:
                        self.count_read_data = 0

                    self.socket.sendall(data)  # send it over TCP

                if self.fin:
                    time.sleep(self.sleeptime)
            except socket.error, msg:
                print msg
                # probably got disconnected
                break
            except IOError, msg:
                print(msg)
                break
        self.alive = False
        if self.fin:
            self.fin.close()

    def writer(self):
        """loop forever and copy socket->serial"""
        while rospy.is_shutdown():
            try:
                data = self.socket.recv(16)
                if not data:
                    break
                self.serial.write(data)  # get a bunch of bytes and send them

                self.count_write_data += 1
                if not (self.count_write_data % self.max_write_data_print):
                    print("write %d data." % self.count_write_data)
                    sys.stdout.flush()
                if self.count_write_data >= sys.maxint - 2:
                    self.count_write_data = 0
            except socket.error, msg:
                print(msg)
                # probably got disconnected
                break
        self.alive = False
        self.thread_read.join()


class PassThroughOptionParser(OptionParser):
    def error(self, msg):
        pass


def initialize_options():
    global parser, options, args
    parser = PassThroughOptionParser(usage="""\
        %prog [options] [port [baudrate]]
        Simple Serial to Network (TCP/IP) redirector.

        Note: no security measures are implemeted. Anyone can remotely connect
        to this service over the network.
        Only one connection at once is supported. When the connection is terminated
        it waits for the next connect.
        """)
    parser.add_option("-p", "--port", dest="port",
                      help="port, a number (default 0) or a device name (deprecated option)",
                      default=None)
    parser.add_option("-b", "--baud", dest="baudrate", action="store",
                      type='int',
                      help="set baudrate, default 9600", default=9600)
    parser.add_option("", "--parity", dest="parity", action="store",
                      help="set parity, one of [N, E, O], default=N",
                      default='N')
    parser.add_option("", "--rtscts", dest="rtscts", action="store_true",
                      help="enable RTS/CTS flow control (default off)",
                      default=False)
    parser.add_option("", "--xonxoff", dest="xonxoff", action="store_true",
                      help="enable software flow control (default off)",
                      default=False)
    parser.add_option("", "--cr", dest="cr", action="store_true",
                      help="do not send CR+LF, send CR only", default=False)
    parser.add_option("", "--lf", dest="lf", action="store_true",
                      help="do not send CR+LF, send LF only", default=False)
    parser.add_option("", "--rts", dest="rts_state", action="store", type='int',
                      help="set initial RTS line state (possible values: 0, 1)",
                      default=None)
    parser.add_option("", "--dtr", dest="dtr_state", action="store", type='int',
                      help="set initial DTR line state (possible values: 0, 1)",
                      default=None)
    parser.add_option("-q", "--quiet", dest="quiet", action="store_true",
                      help="suppress non error messages", default=False)
    parser.add_option("-s", "--sniff", dest="sniff", action="store_true",
                      help="output read data to stdout", default=False)
    parser.add_option("-P", "--localport", dest="local_port", action="store",
                      type='int',
                      help="local TCP port", default=7777)
    parser.add_option("", "--file",
                      help="fake input by reading lines from FILE",
                      metavar="FILE", action="store",
                      type="string", dest="filename")
    parser.add_option("", "--freq",
                      help="frequency of line reading when --file is used",
                      action="store", type="float", dest="freq")
    # Parsing only known arguments
    (options, args) = parser.parse_args()


def assert_options_are_valid():
    global baudrate, port
    if args:
        if options.port is not None:
            parser.error(
                "no arguments are allowed, options only when --port is given")
        args.pop(0)
        if args:
            try:
                baudrate = int(args[0])
            except ValueError:
                parser.error("baudrate must be a number, not %r" % args[0])
            args.pop(0)
        if args:
            parser.error("too many arguments")
    else:
        if port is None:
            port = 0
    if options.cr and options.lf:
        parser.error("ony one of --cr or --lf can be specified")
    if options.filename or options.freq:
        if (not (options.filename)) or (not (options.freq)):
            parser.error("--file and --freq must always be used together")
        else:
            if options.freq <= 0.0:
                parser.error("Freq must be > 0 Hz")

            if not os.path.exists(options.filename):
                parser.error("File '%s' does not exist" % options.filename)


def init_serial_interface():
    try:
        ser.open()
    except serial.SerialException, e:
        print "Could not open serial port %s: %s" % (ser.portstr, e)
        sys.exit(1)
    if options.rts_state is not None:
        ser.setRTS(options.rts_state)
    if options.dtr_state is not None:
        ser.setDTR(options.dtr_state)


def init_tcp_interface():
    global srv
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('', options.local_port))
    srv.listen(1)


if __name__ == '__main__':

    initialize_options()

    port = options.port
    baudrate = options.baudrate

    assert_options_are_valid()

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.rtscts = options.rtscts
    ser.xonxoff = options.xonxoff
    ser.timeout = 1  # required so that the reader thread can exit

    if not options.quiet:
        print "--- TCP/IP to Serial redirector --- type Ctrl-C / BREAK to quit"
        print "--- %s %s,%s,%s,%s ---" % (
            ser.portstr, ser.baudrate, 8, ser.parity, 1)
        if options.filename:
            print "--- reading from %s at %.2f Hz ---" % (
                options.filename, options.freq)

    init_tcp_interface()

    # Initting ROS node
    rospy.init_node('tcp_serial_redirect', anonymous=True)

    while not rospy.is_shutdown():
        try:
            print("Waiting for connection on %s..." % options.local_port)
            connection, addr = srv.accept()
            print('Connected by', addr)
            init_serial_interface()
            sys.stdout.flush()
            r = TCPSerialRedirector(ser, connection, options.filename, options.freq,
                                    options.sniff)
            time.sleep(1)
            r.shortcut()
            ser.close()
            connection.close()
            print('Disconnected from', addr)
        except socket.error, msg:
            print(msg)
        except rospy.ROSInterruptException:
            break

    print("\n--- exit ---")
