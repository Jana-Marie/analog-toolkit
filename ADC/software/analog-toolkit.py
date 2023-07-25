#! /usr/bin/env nix-shell
#! nix-shell -i python3.11 -p python311 python311Packages.pyserial python311Packages.numpy python311Packages.plotext
import socket
import time, random, sys, select
import serial
import struct
from collections import deque
import threading
import curses
import tty
import termios
import plotext as plt

class NonBlockingConsole(object):
	def __enter__(self):
		self.old_settings = termios.tcgetattr(sys.stdin)
		tty.setcbreak(sys.stdin.fileno())
		return self

	def __exit__(self, type, value, traceback):
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


	def get_data(self):
		if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
			return sys.stdin.read(1)
		return False

class ATK():
	def __init__(self, serport):
		self.ser = serial.Serial(serport, "115200")
		
		self.rawBuf = deque()
		self.rawBufLen = 5
		self.running = False
		self.lines = 0
		self.lines_sec = 0
		self.errors = 0
		self.errors_sec = 0
		self.last_broken_line = ''

		self.adc_lsb = 4096 # lsb
		self.v_ref = 3.3 # V

		self.last_printed = 0

	def update():
		if len(self.rawBuf)==8:
			data.append(float(self.rawBuf))
			xdata = np.array(data, dtype='float64')
			self.curve.setData(xdata)
			self.app.processEvents()

	def run(self):
		self.running = True
		threading.Thread(target = self.report).start()
		threading.Thread(target = self.serial_receive_data).start()

	def serial_receive_data(self):
		while self.running:
			if self.ser.inWaiting() > 512:
				try:
					msg = self.ser.read(self.ser.inWaiting()).decode("utf-8")
					lines = msg.split('\r\n')
				except:
					lines = []
				
				try:
					if len(lines[1].replace('\x00','').split(',')) != 9:
						lines[0] = self.last_broken_line + lines[0]
				except:
					pass

				try:
					if len(lines[-1].replace('\x00','').split(',')) != 9:
						self.last_broken_line = lines[-1].replace('\x00','')
						del lines[-1]
				except:
					pass

				for line in lines:
					values = self.parse_line(line)
					if values != None and len(values) > 8:
						self.rawBuf.append(values)
						self.lines += 1
						while len(self.rawBuf) > self.rawBufLen:
							self.rawBuf.popleft()
					elif values != None and len(values) < 8 and len(values) > 1:
						self.errors += 1

	def exit(self):
		self.running = 0

	def set_sample_speed(self, khz):
		self.send_command(str.encode("SPS{}\n".format(khz)))

	def apply_config(self):
		self.send_command(str.encode("SET \n"))

	def set_sample_speed_and_apply(self, khz):
		self.set_sample_speed(khz)
		self.apply_config()

	def send_command(self, command):
		self.ser.write(command)

	def report(self):
		while self.running:
			self.errors_sec = (0.3 * self.errors) + (0.7 * self.errors_sec)
			self.errors = 0
			self.lines_sec = (0.3 * self.lines) + (0.7 * self.lines_sec)
			self.lines = 0
			time.sleep(1)

	def parse_line(self, line):
		if len(line) > 20 and len(line) < 56:
			values = line.replace('\x00','').split(',')
			return values
		return

	def lsb_to_v(self, code):
		return ((self.v_ref * code) / self.adc_lsb)

	def drv5055_v_to_mT(self, v, sens, offs):
		return ((offs - v) * sens)

	def drv5055_lsb_to_mT(self, code, sens, cal):
		offs = 1.650+cal
		return self.drv5055_v_to_mT(self.lsb_to_v(code), sens, offs)

	def update_buf_len(self, len):
		self.rawBufLen = len

	def return_all_channels(self):
		return self.rawBuf[-1]

	def print_report(self, str2):
		try:
			print('\r\033[2A\033[K', int(self.rawBuf[-1][0]),\
									"{0:.3f}V".format(self.lsb_to_v(int(self.rawBuf[-1][1]))),\
									"{0:.3f}V".format(self.lsb_to_v(int(self.rawBuf[-1][2]))),\
									"{0:.3f}V".format(self.lsb_to_v(int(self.rawBuf[-1][3]))),\
									"{0:.3f}V".format(self.lsb_to_v(int(self.rawBuf[-1][4]))),\
									"{0:.3f}V".format(self.lsb_to_v(int(self.rawBuf[-1][5]))),\
									"{0:.3f}V".format(self.lsb_to_v(int(self.rawBuf[-1][6]))),\
									"{0:.3f}V".format(self.lsb_to_v(int(self.rawBuf[-1][7]))),\
									"{0:.3f}V".format(self.lsb_to_v(int(self.rawBuf[-1][8]))),\
									"{0:.3f}V".format(self.lsb_to_v(int(self.rawBuf[-1][9]))),\
									'  Errors', int(self.errors_sec), 'per s; Lines', int(self.lines_sec), 'per s; Loss', int((self.errors_sec+1)/(self.errors_sec+self.lines_sec+1)*100), '%', end='')
			print('\r\033[1B\033[K', str2, end='')
			print()
		except:
			pass

	def return_channels(self, channels): # as int
		try:
			ret = []
			if self.rawBuf[-1][channels[0]] != self.last_printed:
				self.last_printed = self.rawBuf[-1][channels[0]]
				for chan in channels:
					ret.append(int(self.rawBuf[-1][chan]))
				return list(ret)
			return list([None]*len(channels))
		except:
			return list([None]*len(channels))

	def print_channels(self, channels):
		try:
			if self.rawBuf[-1][channels[0]] != self.last_printed:
				self.last_printed = self.rawBuf[-1][channels[0]]
				for chan in channels:
					print(self.rawBuf[-1][chan], end=',')
				print()
		except:
			pass

if(len(sys.argv) < 2):
	print("Usage: python -i analog-toolkit.py <port>")
	exit()

toolkit = ATK(sys.argv[1])
toolkit.run()
time.sleep(0.1)

menu = "\033[4mS\033[0mample Rate | \033[4mA\033[0mpply | \033[4mQ\033[0muit"

print()

with NonBlockingConsole() as nbc:
	# ================ START USER SETUP ================
	plt.title("Magnetic Flux Density")
	plt.ylabel("mT")
	plt.theme('mature')
	mT = deque()
	ts = deque()
	mTlen = 220
	while True:
		# ================ START USER CODE ================
		#toolkit.print_report(menu)

		t, code = toolkit.return_channels([0, 1])
		
		if t != None:
			mT.append(toolkit.drv5055_lsb_to_mT(code, 50, -0.06284))
			ts.append(t/1000.0)
			while len(mT) > mTlen:
				mT.popleft()
				ts.popleft()
			#print("Timestamp:", round(ts, 1), "s,  Magnetic flux density:", round(mT, 2), "mT     ", end='\r')

			plt.cld()

			plt.plot(ts, mT)
			plt.show()
		
		# ================ END USER CODE ================
		inp = nbc.get_data()
		if inp == 'S' or inp == 's':
			#print('Sample Rate (1Hz - 100000Hz): ', end='')
			sample_rate = input()
			if int(sample_rate) >= 1 and int(sample_rate) <= 100000:
				toolkit.set_sample_speed(sample_rate)
			#print('\r\033[KSet to', sample_rate, end='')
		elif inp == 'A' or inp == 'a':
			#print('\r\033[KApplying', end='')
			toolkit.apply_config()
		elif inp == 'Q' or inp == 'q':
			#print('\r\033[KExit', end='')
			toolkit.exit()
			break
		time.sleep(0.0001)