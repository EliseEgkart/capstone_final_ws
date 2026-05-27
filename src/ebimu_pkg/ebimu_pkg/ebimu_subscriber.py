#This is the code for the wireless sensor (ebimu24g)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

sid = 0
imu_data = []

def data_parser(msg_data):
	global sid
	global imu_data

	words = msg_data.split(",")    # Fields split
	if(-1 < words[0].find('-')) :
		chid = words[0]
		chid = chid.split('-')
		sid = int(chid[1])
		words = words[1:]
		imu_data = list(map(float, words)) # float type


class EbimuSubscriber(Node):

	def __init__(self):
		super().__init__('ebimu_subscriber')
		qos_profile = QoSProfile(depth=10)
		self.subscription = self.create_subscription(String, 'ebimu_data', self.callback, qos_profile)
		self.subscription   # prevent unuse variable warning

	def callback(self, msg):
		global sid
		global imu_data

		data_parser(msg.data)
		print(sid, end='')
		print(imu_data)



def main(args=None):
	rclpy.init(args=args)

	print("Starting ebimu_subscriber..")

	node = EbimuSubscriber()

	try:
		rclpy.spin(node)

	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

