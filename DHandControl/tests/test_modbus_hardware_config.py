"""Test all driver motion entries against real configuration, memory Modbus only."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from modbus_dev import DexHandControl
from mh6_hardware_config import HardwareConfig
from mh6_teleop_run import HardwareSender


class MemoryClient:
    connected = True
    def __init__(self):
        self.registers = {}
        self.writes = []
        self.connect = Mock(side_effect=AssertionError("no hardware connection"))
        self.close = Mock()

    def write_register(self,address,value,**kwargs):
        self.writes.append((address,value))
        self.registers[address] = value
        if address == 0:
            kind = self.registers.get(1,0)
            self.registers[5] = {1:0xA0 if kind == 0 else 0xB0,
                                2:0xC0 if kind == 0 else 0xD0,4:0x90,7:0x93,8:0x94}[value]
            if value == 7:
                values = [self.registers[2],1,20,20,30,100,0,0,0,0,0,1]
                self.registers.update({60+i:x for i,x in enumerate(values)})
            if value == 8:
                values = [self.registers[2],1,400,30,7400,0,1,1]
                self.registers.update({72+i:x for i,x in enumerate(values)})
        return SimpleNamespace(isError=lambda:False)

    def write_registers(self,address,values,**kwargs):
        self.writes.append((address,list(values)))
        self.registers.update({address+i:x for i,x in enumerate(values)})
        return SimpleNamespace(isError=lambda:False)

    def read_holding_registers(self,address,count,**kwargs):
        return SimpleNamespace(isError=lambda:False,registers=[self.registers.get(address+i,0) for i in range(count)])


def hand(config=None, *, enabled=True):
    client = MemoryClient()
    with patch("modbus_dev.ModbusClient",return_value=client):
        driver = DexHandControl(port="memory-only",hardware_config=config,enable_motion=enabled)
    driver.persistent_connection = True
    return driver


class DriverConfigTest(unittest.TestCase):
    def test_sender_passes_its_configuration_to_the_locked_driver(self):
        config = HardwareConfig.load()
        sender = HardwareSender("memory-only",115200,hardware_config=config)
        with patch("modbus_dev.DexHandControl") as driver_class:
            driver_class.return_value.start_persistent_connection.return_value = True
            sender.start()
        self.assertIs(driver_class.call_args.kwargs["hardware_config"],config)
        self.assertNotIn("enable_motion",driver_class.call_args.kwargs)
        driver_class.return_value.move_hand.assert_not_called()

    def test_configuration_snapshot_and_custom_limits_are_authoritative(self):
        data = HardwareConfig.load().to_dict()
        data["actuators"]["M2"].update(min=200,max=450,initial_position=400)
        driver = hand(data)
        data["actuators"]["M2"]["max"] = 500
        self.assertFalse(driver.move_palms([2],[451],[80]))
        self.assertEqual(driver.client.writes,[])
        self.assertTrue(driver.move_palms([2],[450],[80]))

    def test_all_motion_entries_obey_default_lock(self):
        data = HardwareConfig.load().to_dict()
        driver = hand(data,enabled=False)
        commands = [lambda:driver.move_fingers([1],[100]),lambda:driver.move_palms([4],[400],[80]),
                    lambda:driver.single_control(0,1,100),lambda:driver.single_control(1,4,400),
                    lambda:driver.move_hand(finger_ids=[1],finger_positions=[100]),
                    lambda:driver.free_all(),lambda:driver.palm_free(),lambda:driver.finger_free(),
                    lambda:driver.move_hand_normalized([0]*5,palm_positions=[300,400,450],m4_value=.5)]
        for command in commands:
            self.assertFalse(command())
            self.assertEqual(driver.last_move_result["status"],"MOTION_LOCKED")
        self.assertEqual(driver.client.writes,[])
        driver.client.connect.assert_not_called()

    def test_user_bounds_are_checked_for_every_single_and_group_axis(self):
        driver = hand()
        for name,spec in driver.hardware_config.actuators.items():
            device_id = int(name[1:]); kind = 0 if name.startswith("F") else 1
            for position in (spec["min"]-1,spec["max"]+1):
                with self.subTest(channel=name,position=position):
                    self.assertFalse(driver.single_control(kind,device_id,position))
                    if kind == 0: self.assertFalse(driver.move_fingers([device_id],[position]))
                    else: self.assertFalse(driver.move_palms([device_id],[position],[80]))
                    self.assertEqual(driver.client.writes,[])
        for name,spec in driver.hardware_config.actuators.items():
            device_id = int(name[1:]); kind = 0 if name.startswith("F") else 1
            for position in (spec["min"],spec["max"]):
                self.assertTrue(driver.single_control(kind,device_id,position))

    def test_invalid_last_axis_cannot_partially_write_combined_command(self):
        driver = hand()
        self.assertFalse(driver.move_hand(finger_ids=[1,2,3,4,5],finger_positions=[20]*5,
            palm_ids=[1,2,3,4],palm_positions=[247,500,500,601],palm_times=[80]*4))
        self.assertEqual(driver.client.writes,[])
        self.assertEqual(driver.last_move_result["status"],"REJECTED")

    def test_empty_duplicate_unknown_boolean_and_noninteger_inputs_are_rejected(self):
        driver = hand()
        commands = [lambda:driver.move_fingers([],[]),lambda:driver.move_palms([],[],[]),
                    lambda:driver.move_fingers([1,1],[20,20]),lambda:driver.move_palms([5],[400],[80]),
                    lambda:driver.move_fingers([True],[20]),lambda:driver.move_fingers([1],[True]),
                    lambda:driver.move_palms([4],[400.0],[80]),lambda:driver.move_palms([4],[400],[True]),
                    lambda:driver.single_control(True,1,300),lambda:driver.move_hand(),
                    lambda:driver.move_hand(palm_ids=[4],palm_positions=[400],palm_times=[-1])]
        for command in commands: self.assertFalse(command())
        self.assertEqual(driver.client.writes,[])

    def test_missing_physical_limit_blocks_low_level_motion(self):
        data = HardwareConfig.load().to_dict()
        data["actuators"]["M4"].update(min=None,max=None)
        driver = hand(data)
        self.assertFalse(driver.move_palms([4],[400],[80]))
        self.assertEqual(driver.client.writes,[])

    def test_complete_packet_preserves_solver_positions_and_includes_m4(self):
        driver = hand()
        self.assertTrue(driver.move_hand_normalized([0,.25,.5,.75,1],
            palm_positions=[300.2,400.4,450.1],m4_value=.5))
        block = driver.client.writes[0][1]
        self.assertEqual(block[0],5)
        self.assertEqual([block[2+i*2] for i in range(5)],[20,502,985,1468,1950])
        self.assertEqual(block[11],4)
        self.assertEqual([block[13+i*3] for i in range(4)],[300,400,450,400])
        self.assertEqual(driver.last_move_result["status"],"BRIDGE_ISSUED")

    def test_bad_normalized_and_removed_palm_mapping_never_write(self):
        driver = hand()
        for invalid in (True,float("nan"),float("inf"),-.1,1.1,"0.5"):
            with self.assertRaises(ValueError): driver.map_m4_position(invalid)
            with self.assertRaises(ValueError): driver.map_finger_positions([invalid]*5)
        with self.assertRaises(ValueError): driver.map_palm_positions([0,0,0])
        with self.assertRaises(ValueError): driver.move_hand_normalized([0]*5,[0]*3)
        self.assertEqual(driver.client.writes,[])

    def test_missing_initial_position_prevents_partial_reset(self):
        data = HardwareConfig.load().to_dict()
        data["actuators"]["M4"]["initial_position"] = None
        driver = hand(data)
        self.assertFalse(driver.free_all())
        self.assertFalse(driver.palm_free())
        self.assertFalse(driver.palm_free([4]))
        self.assertIn("initial_position",driver.last_move_result["error"])
        self.assertEqual(driver.client.writes,[])
        self.assertTrue(driver.finger_free([1,5]))

    def test_free_all_sends_user_initial_positions_for_all_nine_axes(self):
        driver = hand()
        self.assertTrue(driver.free_all())
        block = driver.client.writes[0][1]
        self.assertEqual([block[2+i*2] for i in range(5)],[20]*5)
        self.assertEqual([block[13+i*3] for i in range(4)],[247,500,500,200])
        self.assertEqual([block[14+i*3] for i in range(4)],[2000]*4)

    def test_free_functions_follow_changed_initial_positions_not_mapping_endpoints(self):
        data = HardwareConfig.load().to_dict()
        for i,p in enumerate((300,400,450,300),1):
            data["actuators"][f"M{i}"]["initial_position"] = p
        data["actuators"]["F1"].update(open_position=1950,closed_position=20,initial_position=30)
        driver = hand(data)
        self.assertTrue(driver.free_all())
        block = driver.client.writes[0][1]
        self.assertEqual(block[2],30)
        self.assertEqual([block[13+i*3] for i in range(4)],[300,400,450,300])
        self.assertTrue(driver.palm_free([4]))

    def test_invalid_initial_position_is_rejected_during_config_load(self):
        for name,value in (("M4",199),("M2",501),("F1",19)):
            data = HardwareConfig.load().to_dict()
            data["actuators"][name]["initial_position"] = value
            with self.assertRaises(ValueError): HardwareConfig(data)

    def test_read_feedback_is_available_while_motion_is_locked(self):
        driver = hand(enabled=False)
        self.assertTrue(driver.read_finger_status(1)["query_ok"])
        self.assertTrue(driver.read_palm_status(4)["query_ok"])
        self.assertEqual([value for address,value in driver.client.writes if address == 0],[7,8])


if __name__ == "__main__": unittest.main()
