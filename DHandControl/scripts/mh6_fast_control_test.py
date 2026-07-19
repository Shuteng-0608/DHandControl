#!/usr/bin/env python3
"""Benchmark fast normalized MH6 hand control over persistent Modbus."""

from __future__ import annotations

import argparse
import math
import time
from typing import Optional, Sequence

from modbus_dev import DexHandControl




def main():
    

    mh6 = DexHandControl(port="COM7", baudrate=115200)
   
    
    
    try:
        if not mh6.start_persistent_connection():
            print("ERROR: failed to start persistent Modbus connection")
            return 1
        print("Persistent Modbus connection started. Sending commands...")


        mh6.free_all()


        mh6.move_fingers([1,2,3,4,5],[200,1000,1000,1000,1000])

        time.sleep(2.0)

        mh6.finger_free([1,2,3,4,5])









        












    except Exception as e:
        print(f"ERROR: exception while starting persistent connection: {e}")
        return 1
    
    finally:
        mh6.stop_persistent_connection()





if __name__ == "__main__":
    raise SystemExit(main())
    