from __future__ import division

r"""
RTT.py -- GDB script for setting SEGGER RTT address according to MAP file

Usage: gdb-py --batch -x RTT.py -ex "py RTT(<bin_file>)"
"""

import gdb
import sys
import os
import re

#  J-Link GDB Server
HOST = 'localhost'
PORT = 2331

RE_RTT_ADDR = r'\s+(0x[0-9a-fA-F]+)\s+_SEGGER_RTT\s*'

#  Execute GDB command
def execute( st ):
    return gdb.execute( st, to_string=True )

#  Execute GDB 'monitor' command
def monitor( st ):
    return execute( 'monitor ' + st )

#  Set taken from MAP file address of SEGGER_RTT structure
def set_RTT( fn ):
    fn_map = os.path.splitext( fn )[ 0 ] + '.map'
    if os.path.exists( fn_map ):
        regex = re.compile( RE_RTT_ADDR )
        for ln in open( fn_map ).readlines():
            m = regex.match( ln )
            if m:
                RTT = m.group( 1 )
                print 'RTT structure at', RTT
                monitor( 'exec SetRTTAddr ' + RTT )
                return True
    return False

#  Wrapper for setting RTT address from shell
def RTT( binary ):
    execute( 'set pagination off' )
    print 'J-Link GDB Server connecting...'
    try:
        execute( 'target remote %s:%d' % ( HOST, PORT ))

    except Exception as e:
        print 'Fail to connect.'
        print e.message
        print 'Please start J-Link GDB Server first.'
        return False

    result = set_RTT( binary )

    fb = monitor( 'reset 0' )
    print fb.strip()
    monitor( 'go' )

    return result
