import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool, String
from dynamixel_sdk import *

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_CURRENT = 102
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_CURRENT = 126
ADDR_OPERATING_MODE = 11
ADDR_DRIVE_MODE = 10
MULTI_TURN_ENABLE = 2

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'

class DynamixelMasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.declare_parameter('ids', [11])
        self.dxl_ids = self.get_parameter('ids').get_parameter_value().integer_array_value

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            self.get_logger().error("❌ 포트 열기 실패")
            return
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("❌ 보레이트 설정 실패")
            return

        self.current_mode = {dxl_id: 'position' for dxl_id in self.dxl_ids}
        self.pub = {}
        for dxl_id in self.dxl_ids:
            self.pub[dxl_id] = {
                'position': self.create_publisher(Int32, f'/dxl{dxl_id}/state/position', 10),
                'velocity': self.create_publisher(Int32, f'/dxl{dxl_id}/state/velocity', 10),
                'current': self.create_publisher(Int32, f'/dxl{dxl_id}/state/current', 10),
                'mode': self.create_publisher(String, f'/dxl{dxl_id}/state/mode', 10),
                'torque': self.create_publisher(Bool, f'/dxl{dxl_id}/state/torque_enabled', 10)
            }

        self.group_bulk_read = GroupBulkRead(self.portHandler, self.packetHandler)
        for dxl_id in self.dxl_ids:
            self.set_operating_mode(dxl_id, 'position')
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)

            # bulk read 요청 등록
            self.group_bulk_read.addParam(dxl_id, ADDR_PRESENT_POSITION, 10)  # pos(4) + vel(4) + cur(2)
            
            self.create_subscription(Int32, f'/dxl{dxl_id}/goal_position', lambda msg, i=dxl_id: self.send_command(i, 'position', msg.data), 10)
            self.create_subscription(Int32, f'/dxl{dxl_id}/goal_velocity', lambda msg, i=dxl_id: self.send_command(i, 'velocity', msg.data), 10)
            self.create_subscription(Int32, f'/dxl{dxl_id}/goal_current', lambda msg, i=dxl_id: self.send_command(i, 'current', msg.data), 10)
            self.create_subscription(String, f'/dxl{dxl_id}/set_mode', lambda msg, i=dxl_id: self.set_operating_mode(i, msg.data), 10)

        self.timer = self.create_timer(0.02, self.read_states)
        self.get_logger().info(f"✅ Dynamixel Master Node 시작 (IDs: {self.dxl_ids})")
        
        for dxl_id in self.dxl_ids:
            self.send_command(dxl_id, 'position', 0)

    def send_command(self, dxl_id, mode, value):
        if self.current_mode[dxl_id] != mode:
            return
        if mode == 'position':
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, value)
        elif mode == 'velocity':
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_VELOCITY, value)
        elif mode == 'current':
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_CURRENT, value)

    def set_operating_mode(self, dxl_id, mode):
        mode_value = {'position': 4, 'velocity': 1, 'current': 0}.get(mode, 3)
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, mode_value)
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_DRIVE_MODE, MULTI_TURN_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
        self.current_mode[dxl_id] = mode
        self.pub[dxl_id]['mode'].publish(String(data=mode))
        self.pub[dxl_id]['torque'].publish(Bool(data=True))
        self.get_logger().info(f"🔧 [ID {dxl_id}] 모드 설정 → {mode}")

    def read_states(self):
        if self.group_bulk_read.txRxPacket() != COMM_SUCCESS:
            self.get_logger().warn("⚠️ Bulk Read 실패")
            return

        for dxl_id in self.dxl_ids:
            if not self.group_bulk_read.isAvailable(dxl_id, ADDR_PRESENT_POSITION, 10):
                self.get_logger().warn(f"⚠️ [ID {dxl_id}] position/velocity/current not available")
                continue

            pos = self.group_bulk_read.getData(dxl_id, ADDR_PRESENT_POSITION, 4)
            vel = self.group_bulk_read.getData(dxl_id, ADDR_PRESENT_VELOCITY, 4)
            cur = self.group_bulk_read.getData(dxl_id, ADDR_PRESENT_CURRENT, 2)

            self.pub[dxl_id]['position'].publish(Int32(data=pos))
            self.pub[dxl_id]['velocity'].publish(Int32(data=vel))
            self.pub[dxl_id]['current'].publish(Int32(data=cur))

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelMasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
