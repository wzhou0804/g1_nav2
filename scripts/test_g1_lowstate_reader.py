"""Exercise the real SDK receive path on loopback/domain 120; no motor commands."""
from pathlib import Path
import sys
import time
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from unitree_sdk2py.core import channel
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from scripts.deploy_g1_student import StateReader
from navigation.homie import Configuration
from navigation.student_deploy_core import DeployFault,validate_state


class LowStateReception(unittest.TestCase):
    def test_burst_receive_pause_duplicates_and_resume(self):
        channel.ChannelConfigHasInterface=(
            '<CycloneDDS><Domain><General><Interfaces>'
            '<NetworkInterface address="$__IF_NAME__$"/></Interfaces>'
            '<AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>')
        channel.ChannelFactoryInitialize(120,'127.0.0.1')
        publisher=channel.ChannelPublisher('rt/lowstate',LowState_);publisher.Init()
        reader=StateReader(domain_id=120)
        crc=CRC();cfg=Configuration();cfg.require_l1=False
        message=unitree_hg_msg_dds__LowState_();message.mode_machine=6
        message.imu_state.quaternion=[1.,0.,0.,0.]
        def send(tick):
            message.tick=tick;message.crc=crc.Crc(message)
            self.assertTrue(publisher.Write(message))
        def await_tick(tick):
            deadline=time.monotonic()+2.
            while time.monotonic()<deadline:
                s=reader.get()
                if s is not None and s.tick==tick:return s
                time.sleep(.002)
            self.fail(f'Latest tick {tick} not received: {reader.diagnostics()}')
        try:
            time.sleep(.1)
            for tick in range(1,201):send(tick)
            validate_state(await_tick(200),time.monotonic(),cfg)
            first=reader.get().stamp
            for _ in range(30):
                send(200);time.sleep(.005)
            self.assertEqual(reader.get().stamp,first)
            with self.assertRaisesRegex(DeployFault,'LowState stale'):validate_state(reader.get(),time.monotonic(),cfg)
            stats=reader.diagnostics()
            self.assertGreater(stats['duplicate_ticks'],0)
            self.assertGreater(stats['unique_tick_age_ms'],100.)
            self.assertLess(stats['last_message_age_ms'],100.)
            send(201)
            validate_state(await_tick(201),time.monotonic(),cfg)
            time.sleep(.13)
            with self.assertRaisesRegex(DeployFault,'LowState stale'):validate_state(reader.get(),time.monotonic(),cfg)
            send(202);validate_state(await_tick(202),time.monotonic(),cfg)
        finally:
            reader.close();publisher.Close()
        self.assertFalse(reader.thread.is_alive())


if __name__=='__main__':unittest.main()
