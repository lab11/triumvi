Wireless Metering V8
====================

Main meter application.

Setup 
-----

1. Make sure you have contiki checked out in the same folder as the G2 repository.
It must be on this branch:

        https://github.com/lab11/contiki/tree/triumvi


Compile
-------

You must point to the correct calibration file:

    make calFile='\"calibrate/V8node12SineTable.txt\"'
    
To flash the device:

    make ID=c0:98:e5:54:52:a0:00:01 calFile='\"calibrate/V8node12SineTable.txt\"' install
