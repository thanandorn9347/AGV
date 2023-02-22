#!/usr/bin/env python3

import pyfirmata
import time

if __name__ == '__main__':
    board = pyfirmata.Arduino('/dev/ttyACM1')
    print("Communication Successfully started")
    print(board.get_firmata_version())
    pin3 = board.get_pin('d:3:i')
    test= pin3.read()
    it = pyfirmata.util.Iterator(board)
    it.start()
    pinA0 = board.get_pin('a:0:i')
    # pinA0.enable_reporting()
    test2= pinA0.read()
    while True:
        board.digital[13].write(1)
        time.sleep(1)
        board.digital[13].write(0)
        time.sleep(1)