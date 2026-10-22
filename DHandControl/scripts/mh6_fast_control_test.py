#!/usr/bin/env python3
"""建立持久连接并清错，在 main 中直接编写测试指令。"""
import time

from modbus_dev import DexHandControl


def main():
    mh6 = None
    try:
        mh6 = DexHandControl(
            port="/dev/ttyUSB0", baudrate=115200, timeout=0.15,
            enable_motion=True,  # 需要发送位置指令时，手动改为 True
        )
        if not mh6.start_persistent_connection():
            raise RuntimeError("串口连接失败")

        # 开机固定流程：清除五个手指的错误；失败时不进入测试。
        for finger_id in range(1, 6):
            if not mh6.clear_error(finger_id):
                raise RuntimeError(f"F{finger_id} 清错失败，停止测试")
        print("持久连接已建立，五指清错完成。")

        # ===== 在这里编写你的测试代码 =====
        # mh6.move_fingers([2], [100])
        # mh6.move_palms([4], [220], [1000])
        mh6.free_all()

        # mh6.move_palms([4],[400],[2000])
        # while True:
        #     for finger_id in range(1, 6):
        #         state = mh6.read_finger_status(finger_id)
        #         print(f"F{finger_id}：", state)
        #         time.sleep(0.1)

        #     for palm_id in range(1, 5):
        #         state = mh6.read_palm_status(palm_id)
        #         print(f"M{palm_id}：", state)
        #         time.sleep(0.1)

        #     print("-" * 60)









        # ================================

        return 0
    except KeyboardInterrupt:
        print("测试中断，关闭连接。")
        return 130
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        if mh6 is not None:
            mh6.stop_persistent_connection()


if __name__ == "__main__":
    raise SystemExit(main())
