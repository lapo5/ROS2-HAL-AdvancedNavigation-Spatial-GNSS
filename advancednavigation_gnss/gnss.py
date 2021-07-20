#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
import cv2
from pymba import *
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped
from allied_vision_camera_interfaces.srv import CameraState
from cv_bridge import CvBridge
import threading
import sys

from std_msgs.msg import Header
import tf2_ros
import geometry_msgs

import json
import signal
import sys
from multiprocessing import Process, Queue
import numpy as np
from math import floor

import datetime, threading, time

from . geotranslator import GeoTranslator

################################### NTRIP SUPPORT #####################################
from base64 import b64encode
from threading import Thread
from http.client import HTTPConnection
from http.client import IncompleteRead
''' This is to fix the IncompleteRead error
	http://bobrochel.blogspot.com/2010/11/bad-servers-chunked-encoding-and.html'''
import http.client
import requests
from requests.auth import HTTPBasicAuth

from scipy.spatial.transform import Rotation as R
import csv
import os


def gen_gga(time_t, lat, lat_pole, lng, lng_pole, fix_quality, num_sats, hdop, alt_m, geoidal_sep_m, dgps_age_sec,
			dgps_ref_id):
	hhmmssss = '%02d%02d%02d%s' % (time_t.tm_hour, time_t.tm_min, time_t.tm_sec, '.%02d' if 0 != 0 else '')

	lat_abs = abs(lat)
	lat_deg = lat_abs
	lat_min = (lat_abs - floor(lat_deg)) * 60
	lat_sec = round((lat_min - floor(lat_min)) * 1000)
	lat_pole_prime = ('S' if lat_pole == 'N' else 'N') if lat < 0 else lat_pole
	lat_format = '%02d%02d.%03d' % (lat_deg, lat_min, lat_sec)

	lng_abs = abs(lng)
	lng_deg = lng_abs
	lng_min = (lng_abs - floor(lng_deg)) * 60
	lng_sec = round((lng_min - floor(lng_min)) * 1000)
	lng_pole_prime = ('W' if lng_pole == 'E' else 'E') if lng < 0 else lng_pole
	lng_format = '%03d%02d.%03d' % (lng_deg, lng_min, lng_sec)

	dgps_format = '%s,%s' % (
	'%.1f' % dgps_age_sec if dgps_age_sec is not None else '', '%04d' % dgps_ref_id if dgps_ref_id is not None else '')

	str = 'GPGGA,%s,%s,%s,%s,%s,%d,%02d,%.1f,%.1f,M,%.1f,M,%s' % (
	hhmmssss, lat_format, lat_pole_prime, lng_format, lng_pole_prime, fix_quality, num_sats, hdop, alt_m, geoidal_sep_m,
	dgps_format)
	crc = 0
	for c in str:
		crc = crc ^ ord(c)
	crc = crc & 0xFF

	return '$%s*%0.2X' % (str, crc)


def NiceToICY(self):
	class InterceptedHTTPResponse():
		pass

	import io
	line = self.fp.readline().replace(b"ICY 200 OK\r\n", b"HTTP/1.0 200 OK\r\n").replace(b"SOURCETABLE 200 OK\r\n", b"HTTP/1.0 200 OK\r\n")
	InterceptedSelf = InterceptedHTTPResponse()
	InterceptedSelf.fp = io.BufferedReader(io.BytesIO(line))
	InterceptedSelf.debuglevel = self.debuglevel
	InterceptedSelf._close_conn = self._close_conn
	return ORIGINAL_HTTP_CLIENT_READ_STATUS(InterceptedSelf)


ORIGINAL_HTTP_CLIENT_READ_STATUS = http.client.HTTPResponse._read_status
http.client.HTTPResponse._read_status = NiceToICY


def patch_http_response_read(func):
	def inner(*args):
		try:
			return func(*args)
		except http.client.IncompleteRead as e:
			return e.partial

	return inner

http.client.HTTPResponse.read = patch_http_response_read(http.client.HTTPResponse.read)

class ntripconnect(Thread):
	def __init__(self, client):
		super(ntripconnect, self).__init__()
		self.client = client
		self.stop = False
		self.lat = None
		self.lon = None
		self.nmea_gga = ""
		self.connection = None
		self.response = None
		self.headers = {
			'Ntrip-Version': 'Ntrip/2.0',
			'User-Agent': 'NTRIP TAS-I O2D',
			'Connection': 'close',
			'Authorization': 'Basic ' + b64encode(
				(self.client.ntrip_user + ':' + self.client.ntrip_pass).encode()).decode("ascii")
		}

	def update_gga(self):
		self.nmea_gga = gen_gga(time.gmtime(),
								abs(self.lat),
								'N' if self.lat >= 0 else 'S',
								abs(self.lon),
								'E' if self.lon >= 0 else 'O',
								1, 12, 1.0, 0.0, 0.0, None, None)

	def connect(self):
		if self.connection is None:
			self.connection = HTTPConnection(self.client.ntrip_server)
		else:
			self.connection.close()
		self.connection.request('GET', '/' + self.client.ntrip_stream, self.nmea_gga, self.headers)
		self.response = self.connection.getresponse()
		if self.response.status != 200:
			raise Exception("Request failed")

	def run(self):
		self.update_gga()
		self.connect()
		while not self.stop:
			try:
				data = self.response.read(1)
				if len(data) != 0:
					if data[0] == 211:                        # Byte 0 | RTCMv3 preamble
						buf = []
						buf.append(data[0])
						data = self.response.read(2)
						buf.append(data[0])
						buf.append(data[1])
						cnt = data[0] * 256 + data[1]         # Byte 1-2 | 6 bits padding (000000) + 10 bit byte count
						data = self.response.read(2)
						buf.append(data[0])
						buf.append(data[1])
						typ = (data[0] * 256 + data[1]) >> 4  # Byte 3-4 | 12 bits message number
						#out = ' '.join([str(datetime.datetime.now()), "Size:", str(cnt), " Type:", str(typ)])
						#print(out)
						cnt = cnt + 1
						for x in range(cnt):
							data = self.response.read(1)
							buf.append(data[0])
						#print(buf)
						self.client.geotrans.hal_gps.put_rtcm_msg(buf, len(buf))
					else:
						pass
				else:
					self.update_gga()
					self.connect()
			except ConnectionResetError as e:
				print("Connection Reset by Peer - Manage")
				self.update_gga()
				self.connect()

		self.connection.close()
######################################################################################


# Class definition of the calibration function
class GNSSNode(Node):

	ntrip_server = "158.102.7.10:2101"
	ntrip_user = "roncapat"
	ntrip_pass = "roncapat"
	ntrip_stream = "RTK_MAC/MAX_RTCM3"

	def __init__(self):
		super().__init__("advancednavigation_gnss_node")
		self.get_logger().info("Advanced Navigation GNSS node is awake...")
	
		# Class attributes
		self.acquire_data = True

		heading_toward_north = 0.0

		self.geotrans = GeoTranslator(heading_toward_north)
		if not self.geotrans.init():
			print("Cannot communicate with {0} GNSS".format(self.geotrans.name))
			sys.exit(1)

		self.geotrans.acquire()
		self.starting_lat = np.float32(self.geotrans.get_starting_lat())
		self.starting_lon = np.float32(self.geotrans.get_starting_lon())
		self.starting_alt = np.float32(self.geotrans.get_starting_alt())


		# NTRIP thread
		self.ntrip_thread = ntripconnect(self)
		self.ntrip_thread.lat = self.starting_lat
		self.ntrip_thread.lon = self.starting_lon
		self.ntrip_thread.daemon = True
		self.ntrip_thread.start()

		# Publishers
		self.gnss_pub = self.create_publisher(NavSatFix, "/GNSS/raw_data", 1)
		self.geotrans_pub = self.create_publisher(PoseStamped, "/GNSS/pose", 1)
		self.timer = self.create_timer(2.0, self.publish_gnss_info)


				
	# This function stops/enable the acquisition stream
	def exit(self):
		self.ntrip_thread.stop = True
		self.ntrip_thread.join()



	def warmup(self, n):
		print("GNS - Warming up...", end='')
		for i in range(n):
			self.geotrans.acquire()
			self.geotrans.reset()
		print(" Done.")

		self.starting_lat = np.float32(self.geotrans.get_starting_lat())
		self.starting_lon = np.float32(self.geotrans.get_starting_lon())


	# Publisher function
	def publish_gnss_info(self):
		self.geotrans.acquire()
		self.ntrip_thread.lat = self.geotrans.res[0]
		self.ntrip_thread.lon = self.geotrans.res[1]

		msg = NavSatFix()
		msg.header = Header()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = "AdvNav_GNSS"
		msg.status = NavSatStatus()
		msg.status.status = 1
		msg.status.service = 7
		msg.latitude = float(self.geotrans.geotedic_pose[0, 0])
		msg.longitude = float(self.geotrans.geotedic_pose[1, 0])
		msg.altitude = float(self.geotrans.geotedic_pose[2, 0])

		msg.position_covariance[0] = self.geotrans.geotedic_confidences[0, 0]
		msg.position_covariance[4] = self.geotrans.geotedic_confidences[1, 0]
		msg.position_covariance[8] = self.geotrans.geotedic_confidences[2, 0]

		msg.position_covariance_type = 2
		self.gnss_pub.publish(msg)

		msg_pose = PoseStamped()
		msg_pose.header = Header()
		msg_pose.header.stamp = self.get_clock().now().to_msg()
		msg_pose.header.frame_id = "AdvNav_GNSS"

		# Translation
		msg_pose.pose.position.x = float(self.geotrans.tf[0, 3])
		msg_pose.pose.position.y = float(self.geotrans.tf[1, 3])
		msg_pose.pose.position.z = float(self.geotrans.tf[2, 3])

		# short-Rodrigues (angle-axis)
		msg_pose.pose.orientation.x = 0.0
		msg_pose.pose.orientation.y = 0.0
		msg_pose.pose.orientation.z = 0.0
		msg_pose.pose.orientation.w = 1.0

		# Publish the message
		self.geotrans_pub.publish(msg_pose)



# Main loop function
def main(args=None):

	rclpy.init(args=args)
	node = GNSSNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		print('Node stopped cleanly')
		node.exit()
	except BaseException:
		print('Exception in node:', file=sys.stderr)
		raise
	finally:
		# Destroy the node explicitly
		# (optional - Done automatically when node is garbage collected)
		node.destroy_node()
		rclpy.shutdown() 


# Main
if __name__ == '__main__':
	main()