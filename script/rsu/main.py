import os
import json
from datetime import datetime
import time
import random

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
            time.sleep(max(0, self._period - (datetime.now() - st).total_seconds()))

    def stop(self):
        self._run = False

@typechecked
class SchedR(Thread):

    def __init__(self, name: str, callback: Callable[[], None]):
        super().__init__(name=name, daemon=True)

        self._callback = callback
        self._run = True

    def run(self):
        while self._run:
            self._callback()

    def stop(self):
        self._run = False

@typechecked
class RSU:

    def __init__(self, fname: str, iface: str, tmsg: List[Type[V2xMsg]]):

        if not os.path.exists(fname):
            raise Exception(f"{fname} not found")

        self._trace = None
        with open(fname, "r+") as f:
            self._trace = json.loads(f.read())

        assert self._trace is not None
        self._idx, self._size = 0, len(self._trace) - 1

        self._net = V2xNetwork(iface, tmsg)
        self._tmsg = tmsg
        
        # for now i don't want to send cam_msg on the rsu
        timer_cam = SchedT("cam_sender", self.send_cam, 0.1)
        timer_denm = SchedR("denm_sender", self.send_denm)
        timer_cam.start()
        timer_denm.start()

    def send_cam(self):
        msg = self._tmsg[V2xTMsg.CAM - 1](**self._trace[self._idx])
        self._net.send_msg(ETSI.format_msg(msg, gn_addr_address="3E:B5:93:C7:D8:57"))
        self._idx = (self._idx + 1) % (self._size)

    def send_denm(self):
        # 1 -> verde, 2 -> giallo, 3 -> rosso
        for sub_code, period in zip([1, 2, 3], [5, 10, 3]):
            time.sleep(period)
            msg = self._tmsg[V2xTMsg.DENM - 1](
                protocolVersion=2,
                messageID=1,
                stationID=12131,
                originatingStationID=12131,
                sequenceNumber=1,
                detectionTime=100000000,
                referenceTime=0,
                latitude=446529860,
                longitude=109299810,
                semiMajorConfidence=282,
                semiMinorConfidence=278,
                semiMajorOrientation=616,    
                altitudeValue=9650,
                altitudeConfidence="alt-020-00",
                validityDuration=1,
                stationType=15,
                situation_informationQuality=4,
                situation_eventType_causeCode=1,
                situation_eventType_subCauseCode=sub_code,
            )
            self._net.send_msg(ETSI.format_msg(msg, gn_addr_address="3E:B5:93:C7:D8:57"))

def main():
    
    cfiles = ["./asn/cam/CAM-PDU-Descriptions.asn", "./asn/cam/cdd/ITS-Container.asn"]
    dfiles = ["./asn/denm/DENM-PDU-Descriptions.asn", "./asn/denm/cdd/ITS-Container.asn"]
    # TODO: IS_ITS è un TS e quindi suca
    # sfiles = ["./asn/is/SPATEM-PDU-Descriptions.asn", "./asn/is/cdd/ITS-Container.asn", "./asn/is/lib/asn1/IS/ISO_TS_19091/original/DSRC.asn"]

    CAM =  V2xAsnP.new("CAM", cfiles).create_class()
    DENM = V2xAsnP.new("DENM", dfiles).create_class()
    # SPATEM = V2xAsnP.new("SPATEM", sfiles).create_class()

    rsu = RSU("rsu.json", "hwsim0", [DENM, CAM])
    while 1: pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
