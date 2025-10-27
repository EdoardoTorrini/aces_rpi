import os
import json
from datetime import datetime
import time

from typing import Type, List, Callable
from threading import Thread
from typeguard import typechecked

from pyv2x import ETSI, V2xTMsg
from pyv2x.v2x_msg import V2xMsg
from pyv2x.v2x_utils import V2xAsnP
from pyv2x.v2x_network import V2xNetwork


@typechecked
class SchedT(Thread):

    def __init__(self, name: str, callback: Callable[[], None], period: float | int):
        super().__init__(name=name, daemon=True)

        self._callback = callback
        self._period = float(period)
        self._run = True

    def run(self):
        while self._run:
            st = datetime.now()
            self._callback()
            time.sleep(self._period - max(0, (datetime.now() - st).total_seconds()))

    def stop(self):
        self._run = False

@typechecked
class RSU:

    def __init__(self, fname: str, tmsg: List[Type[V2xMsg]]):
        self._trace = "placeholder"
        self._idx = 0

        timer_cam = SchedT("cam_sender", self.send_cam, 0.1)
        timer_cam.start()

    def send_cam(self):
        print(f"{self._idx} time: {datetime.now().timestamp()}")
        self._idx += 1

def main():
    
    cfiles = ["./asn/cam/CAM-PDU-Descriptions.asn", "./asn/cam/cdd/ITS-Container.asn"]
    dfiles = ["./asn/denm/DENM-PDU-Descriptions.asn", "./asn/denm/cdd/ITS-Container.asn"]

    CAM =  V2xAsnP.new("CAM", cfiles).create_class()
    DENM = V2xAsnP.new("DENM", dfiles).create_class()

    rsu = RSU()
    while 1: pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
