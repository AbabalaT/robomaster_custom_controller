import pyrealsense2.pyrealsense2 as rs
import serial
import struct
import math
import RPi.GPIO as GPIO
from socket import *
from time import sleep

addr = ('127.0.0.1', 1180)
#头段	
SOF = 0xa5
data_length = 30 #data段长度
seq = 0x00 #包序号，每一次加一
crc8 = 0x00 
#命令字
cmd_id = 0x0302 
#数据段
key_1 = 0xff
key_2 = 0xff

location_x = 0.0
location_y = 0.0
location_z = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0
end_float = float("inf")
#尾校验
crc16 = 0x00

if __name__ == '__main__':
	s = socket(AF_INET, SOCK_DGRAM)
	#uart_com = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(4, GPIO.OUT)
	led_state = False
	led_toggle = 5
	GPIO.output(4, led_state)
	while(1):
		while(1):
			try:
				if not GPIO.input(17): #按键1按下
					break
				else:
					sleep(0.25)
					led_state = True
					led_toggle = 5
					GPIO.output(4, led_state)
			except KeyboardInterrupt:
				exit()
		try:
			pipe = rs.pipeline()
			cfg = rs.config()
			cfg.enable_stream(rs.stream.pose)
			pipe.start(cfg)
		except:
			print('start fail, retrying!')
			continue
		while(not GPIO.input(17)): #按键1按下
			try:
				try:	
				    frames = pipe.wait_for_frames()
				except:
				    print('Device disconnected!')
				    pipe.stop()
				    break
				pose = frames.get_pose_frame()
				if pose:
					led_toggle = led_toggle - 1
					if led_toggle < 0:
						led_toggle = 5
						if led_state:
							led_state = False
						else:
							led_state = True
						GPIO.output(4, led_state)
					#print(seq)
					data = pose.get_pose_data()
					w = data.rotation.w
					x = -data.rotation.z
					y = data.rotation.x
					z = -data.rotation.y
					pitch = -math.asin(2.0 * (x*z - w*y)) * 180.0 / math.pi + 15.0
					roll =  math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / math.pi - 40.0
					yaw = math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / math.pi
					
					if GPIO.input(18):
						location_y = data.velocity.x
						location_x = -data.velocity.z
						location_z = data.velocity.y
					else:
						location_y = 0
						location_x = 0
						location_z = 0
					if GPIO.input(27):
						key_1 = 0x00
					else:
						key_1 = 0xff
					if GPIO.input(22):
						key_2 = 0x00
					else:
						key_2 = 0xff
						
					seq=seq+1
					if(seq > 255):
						seq = 0
						
					head = struct.pack("<BHB",SOF, data_length, seq)
					head = head + struct.pack("<B", 0x5F)
					#buff = buff + struct.pack("<H", crc16_check(buff))
					buff = head + struct.pack("<HBBfffffff",cmd_id,key_1,key_2,location_y,location_x,location_z,roll,pitch,yaw,end_float)
					buff = buff + struct.pack("<H", 0x5FFF)
					#buff = buff + struct.pack("<H", crc16_check(buff))
					#print(location_x, location_y, location_z, roll, pitch, yaw)
					print(buff)
					#uart_com.write(buff)
					s.sendto(buff, addr)
					sleep(0.04)
			except KeyboardInterrupt:
				GPIO.output(4, False)
				pipe.stop()
				exit()
		pipe.stop()
		GPIO.output(4, False)
