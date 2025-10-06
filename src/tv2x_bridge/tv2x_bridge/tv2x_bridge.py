import rclpy
from rclpy.node import Node

from datetime import datetime
import os
import json
import asn1tools

from pyv2x.etsi import ETSI
from pyv2x.v2x_utils import GeoNetworking
from pyv2x.v2x_network import V2xNetwork


class V2XBridge(Node):

    def __init__(self):
        super().__init__("tv2x_bridge")
        
        self.load_parameters()
        self._timer = self.create_timer(0.1, self._send_cam)

        self._idx, self._st_time = 0, datetime.now()
        self.get_logger().info(f"starting time: {self._st_time}")

        self._net = V2xNetwork(interface=self._iface)
        self._ok = True
        
    def get_param(self, name: str, default):
        self.declare_parameter(name, default)
        param = self.get_parameter(name).value
        self.get_logger().info(f"get new param - name: {name}, value: {param}")
        return param

    def load_parameters(self):
        self._iface = self.get_param("general.v2x_interface", "")
        self._mac = self.get_param("general.mac", "")

        path = self.get_param("general.cam_asn", "")
        self._asn_cam = asn1tools.compile_files(path, "uper")
        
        path = self.get_param("general.denm_asn", "")
        self._asn_denm = asn1tools.compile_files(path, "uper") 
        
        path = self.get_param("general.trace", "")
        with open(path, "r+") as f:
            self._trace = json.loads(f.read())

  def _format_cam(self, val: dict):
        # {"latitude": 446611558, "longitude": 109342462, "heading": 0, "speed": 0, "delta_time": 21663, "elevation": 6050}
        try:
            return ETSI.new_cam(
                self._asn_cam, gn_addr_address=self._mac, station_id=4316,
                latitude=val["latitude"], longitude=val["longitude"],
                delta_time=val["delta_time"], speed=val["speed"], heading=val["heading"]
            )
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            return None
 
    def _send_cam(self):
        if (datetime.now() - self._st_time).total_seconds() < 10:
            return

        if self._ok:
            pkt = self._format_cam(self._trace[self._idx])
            self._net.send_msg(pkt)
            self._idx += 1 % len(self._trace)
        
        # TODO: else resend the last position with speed: 0.0

def main(args=None):
    rclpy.init(args=args)

    try:
        tv2x_bridge = V2XBridge()
        rclpy.spin(tv2x_bridge)
    except KeyboardInterrupt:
        pass

    if tv2x_bridge is not None:
        tv2x_bridge.destroy_node()

    if rclpy.ok(): rclpy.shutdown()
