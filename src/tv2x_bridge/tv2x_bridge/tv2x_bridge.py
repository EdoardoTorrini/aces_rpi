import rclpy
from rclpy.node import Node

from datetime import datetime
from enum import Enum
import os
import json

from pyv2x.etsi import ETSI, V2xTMsg
from pyv2x.v2x_utils import V2xAsnP, GeoNetworking
from pyv2x.v2x_network import V2xNetwork
from pyv2x.v2x_msg import V2xMsg

import importlib
from etsi_its_cam_msgs.msg import CAM
from etsi_its_denm_msgs.msg import DENM
from std_msgs.msg import Int8

from haversine import haversine, Unit


class ADStatus(Enum):
    GO = 1
    SLOW_DOWN = 2
    STOP = 3


class V2XBridge(Node):

    def __init__(self):
        super().__init__("tv2x_bridge")

        self.load_parameters()

        # --------------------- timer ---.-------------------
        self._ctimer = self.create_timer(0.1, self._send_cam)
        self._rtimer = self.create_timer(0.1, self._rcv_pkt)

        # --------------------- publisher -------------------
        self._pub_cam = self.create_publisher(CAM, self._ctopic, 10)
        self._pub_denm = self.create_publisher(DENM, self._dtopic, 10)
        self._pub_status = self.create_publisher(Int8, self._stopic, 10)

        # --------------------- set up env ------------------
        self._idx, self._st_time = 0, datetime.now()
        self._net = V2xNetwork(self._iface, [self.DENM, self.CAM], filter=f"its && wlan.sa != {self._mac}")
        self._status = ADStatus.GO.value
        self._pub_status.publish( Int8(data=ADStatus.GO.value) )

        self.get_logger().info(f"starting time: {self._st_time}")

    def get_param(self, name: str, default):
        self.declare_parameter(name, default)
        param = self.get_parameter(name).value
        self.get_logger().info(f"get new param - name: {name}, value: {param}")
        return param

    def load_parameters(self):
        self._name = self.get_param("general.name", "")
        self._enable_pub = self.get_param("general.enable_v2x_pub", False)
        self._iface = self.get_param("general.v2x_interface", "")
        self._mac = self.get_param("general.mac", "")

        cfiles = self.get_param("general.cam_asn", ["", ""])
        self.CAM = V2xAsnP.new("CAM", cfiles).create_class()

        dfiles = self.get_param("general.denm_asn", ["", ""])
        self.DENM = V2xAsnP.new("DENM", dfiles).create_class()

        fpath = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(fpath, self.get_param("general.trace", ""))
        with open(path, "r+") as f:
            self._trace = json.loads(f.read())

        self._ctopic = self.get_param("topic.cam", "")
        self._dtopic = self.get_param("topic.denm", "")
        self._stopic = self.get_param("topic.status", "")

        self._radius = self.get_param("adas.radius", 50)
        self._step = self.get_param("adas.min_look_forward", 1)

    def _send_cam(self):
        if (datetime.now() - self._st_time).total_seconds() < 10:
            return
        
        cam = self.CAM(**self._trace[self._idx])
        if self._status in [ ADStatus.GO.value, ADStatus.SLOW_DOWN ]:
            self._idx = (self._idx + 1) % (len(self._trace) - 1)
        else:
            cam.speedValue = 0

        # TODO: else resend the last position with speed: 0.0
        pkt = ETSI.format_msg(cam, gn_addr_address=self._mac)
        self._net.send_msg(pkt)

    def _rcv_pkt(self):
        pkt = self._net.get_new_msg()
        if pkt is None:
            return

        match pkt.get_id():
            case V2xTMsg.DENM:
                msg = self._v2x_denm_to_ros(pkt)
                self._stop_sem(pkt)
                self._send_v2x_msg(self._pub_cam, msg)
            case V2xTMsg.CAM:
                msg = self._v2x_cam_to_ros(pkt)
                self._send_v2x_msg(self._pub_denm, msg)

    def _stop_sem(self, msg):
        # compute the distance between the two point
        pos1 = (msg.latitude * 1e-7, msg.longitude * 1e-7)
        pos2 = (self._trace[self._idx].get("latitude") * 1e-7, self._trace[self._idx].get("longitude") * 1e-7)

        i_next = (self._idx + self._step) % (len(self._trace) - 1)
        pos3 = (self._trace[i_next].get("latitude") * 1e-7, self._trace[i_next].get("longitude") * 1e-7)
        d1, d2 = haversine(pos1, pos2, Unit.METERS), haversine(pos1, pos3, Unit.METERS)
        
        # if d3 is greater than 0 it means that the vehicles is getting closer to the semaphore
        # if d3 is less than 0 it means that the vehicles is getting far from the the semaphore
        d3 = d1 - d2
        if d1 > self._radius or d2 > self._radius or d3 < 0:
            if status != self._status:
                self._pub_status.publish( Int8(data=ADStatus.GO.value) )
                self._status = ADStatus.GO.value
            return
        
        fpath, status = "denm.situation.eventType.subCauseCode", None
        for i in range(1, len(fpath.split(".")) + 1):
            p = ".".join( fpath.split(".")[-i:] )
            if p in dict(msg).keys():
                status = dict(msg).get(p)
                break
        else:
            return

        if status != self._status:
            self._pub_status.publish( Int8(data=status) )
            self._status = status


    def _send_v2x_msg(self, pub, msg):
        if self._enable_pub:
            pub.publish(msg)            

    def _v2x_cam_to_ros(self, pkt):

        etsi_its_cam = importlib.import_module('etsi_its_cam_msgs.msg')

        msg = etsi_its_cam.CAM()
        msg.header = etsi_its_cam.ItsPduHeader( protocol_version=pkt.protocolVersion, message_id=pkt.messageID, station_id=etsi_its_cam.StationID(value=pkt.stationID) )

        # TODO: for now all the confidence are hardcoded
        msg.cam = etsi_its_cam.CoopAwareness(
            generation_delta_time=etsi_its_cam.GenerationDeltaTime(value=pkt.generationDeltaTime),
            cam_parameters=etsi_its_cam.CamParameters(
                basic_container=etsi_its_cam.BasicContainer(
                    station_type=etsi_its_cam.StationType(value=pkt.stationType),
                    reference_position=etsi_its_cam.ReferencePosition(
                        latitude=etsi_its_cam.Latitude(value=pkt.latitude),
                        longitude=etsi_its_cam.Longitude(value=pkt.longitude),
                        altitude=etsi_its_cam.Altitude(
                            altitude_value=etsi_its_cam.AltitudeValue(value=pkt.altitudeValue),
                            altitude_confidence=etsi_its_cam.AltitudeConfidence(value=0)
                        ),
                        position_confidence_ellipse=etsi_its_cam.PosConfidenceEllipse(
                            semi_major_confidence=etsi_its_cam.SemiAxisLength(value=pkt.semiMajorConfidence),
                            semi_minor_confidence=etsi_its_cam.SemiAxisLength(value=pkt.semiMinorConfidence),
                            semi_major_orientation=etsi_its_cam.HeadingValue(value=pkt.semiMajorOrientation)
                        )
                    )
                ),
                high_frequency_container=etsi_its_cam.HighFrequencyContainer(
                    choice=0,
                    basic_vehicle_container_high_frequency=etsi_its_cam.BasicVehicleContainerHighFrequency(
                        heading=etsi_its_cam.Heading(
                            heading_value=etsi_its_cam.HeadingValue(value=pkt.headingValue),
                            heading_confidence=etsi_its_cam.HeadingConfidence(value=0)
                        ),
                        speed=etsi_its_cam.Speed(
                            speed_value=etsi_its_cam.SpeedValue(value=pkt.speedValue),
                            speed_confidence=etsi_its_cam.SpeedConfidence(value=0)
                        ),
                        drive_direction=etsi_its_cam.DriveDirection(value=0),
                        vehicle_length=etsi_its_cam.VehicleLength(
                            vehicle_length_value=etsi_its_cam.VehicleLengthValue(value=pkt.vehicleLengthValue),
                            vehicle_length_confidence_indication=etsi_its_cam.VehicleLengthConfidenceIndication(value=0)
                        ),
                        vehicle_width=etsi_its_cam.VehicleWidth(value=pkt.vehicleWidth),
                        longitudinal_acceleration=etsi_its_cam.LongitudinalAcceleration(
                            longitudinal_acceleration_value=etsi_its_cam.LongitudinalAccelerationValue(value=pkt.longitudinalAccelerationValue),
                            longitudinal_acceleration_confidence=etsi_its_cam.AccelerationConfidence(value=0)
                        ),
                        curvature=etsi_its_cam.Curvature(
                            curvature_value=etsi_its_cam.CurvatureValue(value=pkt.curvatureValue),
                            curvature_confidence=etsi_its_cam.CurvatureConfidence(value=0)
                        ),
                        curvature_calculation_mode=etsi_its_cam.CurvatureCalculationMode(value=0),
                        yaw_rate=etsi_its_cam.YawRate(
                            yaw_rate_value=etsi_its_cam.YawRateValue(value=pkt.yawRateValue),
                            yaw_rate_confidence=etsi_its_cam.YawRateConfidence(value=0)
                        )
                    )
                )
            )
        )

        return msg

    def _v2x_denm_to_ros(self, pkt):

        etsi_its_denm = importlib.import_module('etsi_its_denm_msgs.msg')

        msg = etsi_its_denm.DENM()
        msg.header = etsi_its_denm.ItsPduHeader( protocol_version=pkt.protocolVersion, message_id=pkt.messageID, station_id=etsi_its_denm.StationID(value=pkt.stationID) )

        msg.denm = etsi_its_denm.DecentralizedEnvironmentalNotificationMessage(
            management=etsi_its_denm.ManagementContainer(
                action_id=etsi_its_denm.ActionID( originating_station_id=etsi_its_denm.StationID(value=pkt.originatingStationID), sequence_number=etsi_its_denm.SequenceNumber(value=pkt.sequenceNumber) ),
                detection_time=etsi_its_denm.TimestampIts(value=pkt.detectionTime),
                reference_time=etsi_its_denm.TimestampIts(value=pkt.detectionTime),
                event_position=etsi_its_denm.ReferencePosition(
                    latitude=etsi_its_denm.Latitude(value=pkt.latitude),
                    longitude=etsi_its_denm.Longitude(value=pkt.longitude),
                    altitude=etsi_its_denm.Altitude(
                        altitude_value=etsi_its_denm.AltitudeValue(value=pkt.altitudeValue),
                        altitude_confidence=etsi_its_denm.AltitudeConfidence(value=0)
                    ),
                    position_confidence_ellipse=etsi_its_denm.PosConfidenceEllipse(
                        semi_major_confidence=etsi_its_denm.SemiAxisLength(value=pkt.semiMajorConfidence),
                        semi_minor_confidence=etsi_its_denm.SemiAxisLength(value=pkt.semiMinorConfidence),
                        semi_major_orientation=etsi_its_denm.HeadingValue(value=pkt.semiMajorOrientation)
                    )
                ),
                station_type=etsi_its_denm.StationType(value=pkt.stationType),
                validity_duration=etsi_its_denm.ValidityDuration(value=pkt.validityDuration),
            ),
            situation_is_present=True,
            situation=etsi_its_denm.SituationContainer(
                information_quality=etsi_its_denm.InformationQuality(value=pkt.informationQuality),
                event_type=etsi_its_denm.CauseCode(
                    cause_code=etsi_its_denm.CauseCodeType(value=dict(pkt).get("denm.situation.eventType.causeCode")),
                    sub_cause_code=etsi_its_denm.SubCauseCodeType(value=dict(pkt).get("denm.situation.eventType.subCauseCode"))
                )
            )
        )

        return msg


def main(args=None):
    rclpy.init(args=args)

    try:
        tv2x_bridge = V2XBridge()
        rclpy.spin(tv2x_bridge)
    except KeyboardInterrupt:
        pass

    if tv2x_bridge is not None:
        tv2x_bridge.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()
