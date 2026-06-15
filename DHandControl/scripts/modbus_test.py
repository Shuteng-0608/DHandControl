# from DHandControl.scripts.modbus_dev import DexHandControl
from modbus_dev import DexHandControl


mh6 = DexHandControl(port="/dev/ttyUSB0", baudrate=115200)

try:
    mh6.start_persistent_connection()
    # TEST A
    print("TEST_A: 单寄存器写0x00FE")
    for v in [0x00FD, 0x00FE, 0x00FF]:
        print("\nwrite single register 24 =", hex(v))
        try:
            mh6._write_register_checked(24, v, f"write reg24 {hex(v)}")
            print("OK")
        except Exception as e:
            print("FAILED:", type(e).__name__, e)

    # TEST_B
    print("TEST_B: 只用 function 0x10 写一个寄存器")
    for v in [0x00FD, 0x00FE, 0x00FF]:
        print("\nwrite multiple registers addr=24 values=[", hex(v), "]")
        try:
            mh6._write_registers_checked(24, [v], f"write multi reg24 {hex(v)}")
            print("OK")
        except Exception as e:
            print("FAILED:", type(e).__name__, e)
    
    # TEST_C
    print("TEST_C: 完整写 27 寄存器块，但不触发动作")
    for pos in [253, 254, 255, 509, 510, 511]:
        block = [0] * 27
        block[0] = 5
        block[1] = 1
        block[2] = 0
        block[3] = 2
        block[4] = pos
        block[5] = 3
        block[6] = 0
        block[7] = 4
        block[8] = 0
        block[9] = 5
        block[10] = 0

        print("\nwrite full block, pos =", pos, hex(pos))
        try:
            mh6._write_registers_checked(20, block, f"write block pos={pos}")
            print("OK")
        except Exception as e:
            print("FAILED:", type(e).__name__, e)


finally:
    mh6.stop_persistent_connection()