#!/usr/bin/env python
# reflect input bytes to output, printing as it goes

import serial, sys, optparse, time, random

parser = optparse.OptionParser("pattern")
parser.add_option("--baudrate", type='int', default=57600, help='baud rate')
parser.add_option("--delay", type='float', default=0.0, help='delay between lines')
parser.add_option("--pattern", type='str', default='0123456789', help='pattern to send')
parser.add_option("--echo", action='store_true', default=False, help='echo any bytes received')
parser.add_option("--crlf", action='store_true', default=False, help='add crlf')
parser.add_option("--counter", action='store_true', default=False, help='add counter')
parser.add_option("--rtscts", action='store_true', default=False, help='enable rtscts')
parser.add_option("--dsrdtr", action='store_true', default=False, help='enable dsrdtr')
parser.add_option("--xonxoff", action='store_true', default=False, help='enable xonxoff')
parser.add_option("--randlength", action='store_true', default=False, help='use random pattern length')
parser.add_option("--reopen", type='float', default=0, help='reopen every N seconds')

opts, args = parser.parse_args()

if len(args) != 1:
    print("usage: pattern.py <DEVICE>")
    sys.exit(1)

device = args[0]

port = serial.Serial(device, opts.baudrate, timeout=0,
                     dsrdtr=opts.dsrdtr, rtscts=opts.rtscts, xonxoff=opts.xonxoff)
counter = 0
open_t = time.time()
while True:
    try:
        buf = opts.pattern[:]
        if opts.randlength:
            rlen = int(random.uniform(0, len(buf)))
            buf = buf[:rlen]
        if opts.counter:
            buf += "%02u" % (counter % 100)
        if opts.crlf:
            buf += '\r\n'
        port.write(buf)
        port.flush()
        if opts.echo:
            try:
                count = port.inWaiting()
                if count > 0:
                    buf = port.read(count)
                    if len(buf) > 0:
                        sys.stdout.write(buf)
                        sys.stdout.flush()
            except Exception:
                pass
        if opts.delay > 0.0:
            time.sleep(opts.delay)
        if opts.reopen > 0 and time.time() > open_t + opts.reopen:
            port.close()
            port = serial.Serial(device, opts.baudrate, timeout=0,
                                 dsrdtr=opts.dsrdtr, rtscts=opts.rtscts, xonxoff=opts.xonxoff)
            open_t = time.time()
            print("Reopened")
        counter += 1
    except KeyboardInterrupt:
        sys.exit(0)
