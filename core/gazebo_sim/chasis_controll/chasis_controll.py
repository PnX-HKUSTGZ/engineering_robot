#!/usr/bin/env python
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TwistStamped # 导入 TwistStamped

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Move around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, # 可靠性：可靠 (所有消息都会送达)
    history=HistoryPolicy.KEEP_LAST,       # 历史：保留最近的 N 条消息
    depth=10,                              # 深度：保留最近的 10 条消息
    durability=DurabilityPolicy.VOLATILE   # 持久性：易失 (不保留消息给迟到的订阅者)
)

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def main(args=None):	
	# if args is None:  # <-- 这两行可以保留，但不再用于传递给 rclpy.init()
	# 	args = sys.argv

	rclpy.init() # <-- 关键修改：移除 args 参数
	node = rclpy.create_node('teleop_twist_keyboard') # 这里节点名称最好与包名匹配，例如 'chasis_controll'

	# 注意：你在这里创建的 publisher 的消息类型是 Twist，
	# 但后面你实际填充和发布的是 TwistStamped。
	# pub = node.create_publisher(Twist, 'cmd_vel', 	qos_profile) # <-- 这一行有问题

    # *** 关键修改：发布者类型改为 TwistStamped ***
	pub = node.create_publisher(TwistStamped, 'cmd_vel', qos_profile)


	speed = 0.5
	turn = 1.0
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist_stamped_msg = TwistStamped() # 重命名变量以清晰区分
			twist_stamped_msg.header.stamp = node.get_clock().now().to_msg()
			# 通常，机器人底盘的 Twist 消息是在其 'base_link' 或类似坐标系中定义的
			twist_stamped_msg.header.frame_id = 'base_link' # 确保这里设置了正确的 frame_id

			twist_stamped_msg.twist.linear.x = x*speed; twist_stamped_msg.twist.linear.y = y*speed; twist_stamped_msg.twist.linear.z = z*speed;
			twist_stamped_msg.twist.angular.x = 0.0; twist_stamped_msg.twist.angular.y = 0.0; twist_stamped_msg.twist.angular.z = th*turn
			pub.publish(twist_stamped_msg) # 发布 TwistStamped

	except Exception as e:
		print(e)

	finally:
		# 在 finally 块中，也应该发布 TwistStamped 消息来停止机器人
		# 确保 pub 已经创建并且是 TwistStamped 类型
		final_twist_stamped = TwistStamped()
		final_twist_stamped.header.stamp = node.get_clock().now().to_msg()
		final_twist_stamped.header.frame_id = 'base_link' # 保持一致

		final_twist_stamped.twist.linear.x = 0.0; final_twist_stamped.twist.linear.y = 0.0; final_twist_stamped.twist.linear.z = 0.0
		final_twist_stamped.twist.angular.x = 0.0; final_twist_stamped.twist.angular.y = 0.0; final_twist_stamped.twist.angular.z = 0.0
		pub.publish(final_twist_stamped)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)