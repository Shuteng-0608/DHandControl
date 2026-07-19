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
        print(mh6.clear_error(1))
        print(mh6.clear_error(2))
        print(mh6.clear_error(3))
        print(mh6.clear_error(4))
        print(mh6.clear_error(5))
        # mh6.move_fingers([1],[300])
        print(mh6.clear_all_errors())
        print(mh6.read_all_finger_status())
        print(mh6.read_all_finger_target_position())
        print(mh6.read_all_finger_position())
        print(mh6.read_all_finger_current())
        print(mh6.read_all_finger_force())
        print(mh6.read_all_finger_force_raw())
        print(mh6.read_all_finger_temperature())
        print(mh6.read_all_finger_error_code())

    except Exception as e:
        print(f"ERROR: exception while starting persistent connection: {e}")
        return 1
    
    finally:
        mh6.stop_persistent_connection()





if __name__ == "__main__":
    raise SystemExit(main())
    