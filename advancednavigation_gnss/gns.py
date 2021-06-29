import json
import signal
import sys
from multiprocessing import Process, Queue
import numpy as np
from math import floor
from geotranslator import GeoTranslator
import isotp

import datetime, threading, time
import can

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


class CSVWriter():
    filename = None
    fp = None
    writer = None

    def __init__(self, filename):
        self.filename = filename
        self.fp = open(self.filename, 'w+', encoding='utf8')
        self.writer = csv.writer(self.fp, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL, lineterminator='\n')

    def close(self):
        self.fp.close()

    def write(self, elems):
        self.writer.writerow(elems)

    def size(self):
        return os.path.getsize(self.filename)

    def fname(self):
        return self.filename


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
                        #print("BENE")
                        self.client.gns.hal_gps.put_rtcm_msg(buf, len(buf))
                    else:
                        pass
                        #print("MALE1")
                else:
                    #print("MALE2")
                    self.update_gga()
                    self.connect()
            except ConnectionResetError as e:
                print("Connection Reset by Peer - Manage")
                self.update_gga()
                self.connect()

        self.connection.close()
######################################################################################

class GNS(Process):
    outqueue = None
    filter = None
    imu = None
    ts = None

    ntrip_server = "158.102.7.10:2101"
    #ntrip_server = "10.42.0.121:2101"
    ntrip_user = "roncapat"
    ntrip_pass = "roncapat"
    ntrip_stream = "RTK_MAC/MAX_RTCM3"
    #ntrip_stream = "ROXY_F9P"

    def __init__(self, gns_module, heading_toward_north, can_enable=False, time_source=None, record_trace=True, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.outqueue = Queue()
        self.heading_toward_north = heading_toward_north
        self.gns = GeoTranslator(heading_toward_north, gns_module)
        if time_source is not None:
            self.ts = time_source()
        if not self.gns.init():
            print("Cannot communicate with {0} GNSS".format(self.gns.name))
            sys.exit(1)

        self.gns.acquire()
        self.starting_timestamp = time.time()
        self.starting_lat = np.float64(self.gns.get_starting_lat())
        self.starting_lon = np.float64(self.gns.get_starting_lon())
        self.starting_alt = np.float32(self.gns.get_starting_alt())

        self.stop = False
        self.send_on_can = False

        if can_enable:
            self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype)
            self.addr = isotp.Address(addressing_mode=isotp.AddressingMode.Normal_11bits, rxid=0x1AA, txid=0x0AA)
            self.stack = isotp.CanStack(self.bus, address=self.addr, error_handler=self.my_error_handler)
        else:
            self.bus = None
            self.addr = None
            self.stack = None

        self.record_trace = record_trace

        if self.record_trace:
            o2d_dir = "/home/marco/Desktop/O2D/O2D/"
            if not os.path.exists(o2d_dir):
                o2d_dir = "/home/o2d/O2D/Tests/"

            if not os.path.exists(o2d_dir):
                raise Exception("O2D Dir error")

            if not os.path.exists(o2d_dir + "gnss_trace"):
                os.mkdir(o2d_dir + "gnss_trace")

            self.file_csv = CSVWriter(o2d_dir + "gnss_trace" + '/trace_lat_lon.csv')
            self.file_csv2 = CSVWriter(o2d_dir + "gnss_trace" + '/trace_cartesian.csv')
            self.file_csv3 = CSVWriter(o2d_dir + "gnss_trace" + '/trace_covariance.csv')

        self.dtype_msg = np.dtype(
            [('timestamp', np.uint64), ('starting_lat', np.float64), ('starting_lon', np.float64), ('starting_alt', np.float32)])

        self.gnss_starting_time = 1.0
        if self.gns.hal_gps.has_rtcm():
            self.ntrip_thread = ntripconnect(self)


        if self.gns.hal_gps.has_rtcm():
            self.ntrip_thread.lat = self.starting_lat
            self.ntrip_thread.lon = self.starting_lon
            self.ntrip_thread.daemon = True
            self.ntrip_thread.start()

    def my_error_handler(self, error):
        #logging.warning('IsoTp error happened : %s - %s' % (error.__class__.__name__, str(error)))
        pass


    def signal_handler(self, sig=None, frame=None):
        print("{0} - GNS TERMINATED".format(time.time()), file=sys.stderr, flush=True)
        self.outqueue.close()
        self.stop = True
        if self.gns.hal_gps.has_rtcm():
            self.ntrip_thread.stop = True

        if self.send_on_can:
            self.bus.shutdown()

        if self.record_trace:
            self.file_csv.close()
            self.file_csv2.close()
            self.file_csv3.close()

        print("{0} - GNS EXITING".format(time.time()), file=sys.stderr, flush=True)

        try:
            self.gns.shutdown()
        except:
            pass

    def warmup(self, n):
        print("GNS - Warming up...", end='')
        for i in range(n):
            self.gns.acquire()
            self.gns.reset()
        print(" Done.")

        self.starting_lat = np.float32(self.gns.get_starting_lat())
        self.starting_lon = np.float32(self.gns.get_starting_lon())

        print("starting_lat: {0}".format(self.starting_lat))
        print("starting_lon: {0}".format(self.starting_lon))

    def wait_fix(self):
        print("GNS - Waiting fix...", end='')
        while not self.gns.hal_gps.has_fix():
            pass
        print(" Done.")

        self.gns.reset()

        self.starting_lat = np.float32(self.gns.get_starting_lat())
        self.starting_lon = np.float32(self.gns.get_starting_lon())

        print("starting_lat: {0}".format(self.starting_lat))
        print("starting_lon: {0}".format(self.starting_lon))


    def run(self):
        signal.signal(signal.SIGINT, lambda a, b: self.signal_handler(a, b))
        signal.signal(signal.SIGTERM, lambda a, b: self.signal_handler(a, b))

        if self.send_on_can:
            print("Starting CAN publisher")
            self.timerThread2 = threading.Thread(target=self.publisher_gnss_starting)
            self.timerThread2.daemon = True
            self.timerThread2.start()

        if self.record_trace:
            self.timerThread3 = threading.Thread(target=self.trace_gnss)
            self.timerThread3.daemon = True
            self.timerThread3.start()

        while not self.stop:
            if not self.gns.acquire():
                continue
            if self.ts is not None:
                t = self.ts.time_us()
            else:
                t = int(time.time() * 1e6)

            if self.gns.hal_gps.has_rtcm():
                self.ntrip_thread.lat = self.gns.res[0]
                self.ntrip_thread.lon = self.gns.res[1]
            if not self.stop:
                self.outqueue.put((self.gns.tf, t, self.gns.cartesian_confidences))

    def publisher_gnss_starting(self):
        next_call = time.time()
        while True:
            datetime.datetime.now()
            next_call = next_call + self.gnss_starting_time  # 20Hz
            timestamp = time.time()

            list_to_send = np.array([(self.starting_timestamp, self.starting_lat, self.starting_lon, self.starting_alt)],
                                    dtype=self.dtype_msg)

            list_to_send_bytes = list_to_send.tobytes()

            if self.bus is not None:
                try:
                    self.stack.send(list_to_send_bytes)
                    while self.stack.transmitting():
                        self.stack.process()
                        time.sleep(self.stack.sleep_time())
                except Exception as e:
                    pass

    def trace_gnss(self):
        next_call = time.time()
        while True:
            datetime.datetime.now()
            next_call = next_call + 1.0  # 20Hz
            timestamp = time.time()

            r = R.from_matrix(self.gns.tf[0:3, 0:3])
            quat = r.as_quat()

            self.file_csv.write(
                (timestamp, self.gns.geotedic_pose[0, 0], self.gns.geotedic_pose[1, 0], self.gns.geotedic_pose[2, 0]))

            self.file_csv2.write(
                (timestamp, self.gns.tf[0, 3], self.gns.tf[1, 3], self.gns.tf[2, 3],
                 quat[0], quat[1], quat[2], quat[3]))


            self.file_csv3.write(
                (timestamp, self.gns.cartesian_confidences[0, 0], self.gns.cartesian_confidences[1, 1], self.gns.cartesian_confidences[2, 2]))


            time_now = time.time()
            if next_call - time_now > 0:
                time.sleep(next_call - time_now)


class GNS_Recorder(Process):
    gns = None
    path = ""
    end = False

    ntrip_server = "158.102.7.10:2101"
    ntrip_user = "roncapat"
    ntrip_pass = "roncapat"
    ntrip_stream = "RTK_MAC/MAX_RTCM3"

    def __init__(self, gns_module, heading_toward_north, path="dataset_path/", *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.path = path
        self.heading_toward_north = np.float32(heading_toward_north)

        self.gns = GeoTranslator(self.heading_toward_north, gns_module)
        if not self.gns.init():
            print("Cannot communicate with {0} GNSS".format(self.gns.name))
            sys.exit(1)
        self.gns.acquire()
        self.starting_timestamp = time.time()
        self.starting_lat = np.float64(self.gns.get_starting_lat())
        self.starting_lon = np.float64(self.gns.get_starting_lon())
        self.starting_alt = np.float32(self.gns.get_starting_alt())

        self.gnss_starting_time = 1.0
        if self.gns.hal_gps.has_rtcm():
            self.ntrip_thread = ntripconnect(self)

        if self.gns.hal_gps.has_rtcm():
            self.ntrip_thread.lat = self.starting_lat
            self.ntrip_thread.lon = self.starting_lon
            self.ntrip_thread.daemon = True
            self.ntrip_thread.start()

        self.number_of_acquisitions = 0

    def signal_handler(self, sig=None, frame=None):
        print("{0} - GNS TERMINATED".format(time.time()), file=sys.stderr, flush=True)
        self.end = True

    def run(self):
        signal.signal(signal.SIGINT, lambda a, b: self.signal_handler(a, b))
        signal.signal(signal.SIGTERM, lambda a, b: self.signal_handler(a, b))
        self.measures = None
        self.timetags = None
        dt = np.dtype('float32,float32,float32,float32,float32,float32')

        item = np.empty(shape=[1], dtype=dt)
        while not self.end:
            s = time.time()
            ret = self.gns.acquire()
            e = time.time()
            if not ret:
                continue
            t = np.uint64(int(time.time() * 1e6))
            item[0][0] = self.gns.geotedic_pose[0, 0]
            item[0][1] = self.gns.geotedic_pose[1, 0]
            item[0][2] = self.gns.geotedic_pose[2, 0]
            item[0][3] = self.gns.geotedic_confidences[0, 0]
            item[0][4] = self.gns.geotedic_confidences[1, 0]
            item[0][5] = self.gns.geotedic_confidences[2, 0]

            self.number_of_acquisitions = self.number_of_acquisitions + 1

            if self.measures is None:
                self.measures = np.array([item], dtype=dt)
                self.timetags = np.array([t], dtype=np.uint64)
            else:
                self.measures = np.append(self.measures, item)
                self.timetags = np.append(self.timetags, t)
            
            if self.gns.hal_gps.has_rtcm():
                self.ntrip_thread.lat = self.gns.res[0]
                self.ntrip_thread.lon = self.gns.res[1]

        if self.gns.hal_gps.has_rtcm():
            self.ntrip_thread.stop = True

        with open('{0}/gns_info.json'.format(self.path), 'w+') as out_file:
            out_file.write(json.dumps(str(self.starting_lat)))
            out_file.write("\n")
            out_file.write(json.dumps(str(self.starting_lon)))
            out_file.write("\n")
            out_file.write(json.dumps(str(self.starting_alt)))
            out_file.write("\n")
            out_file.write(json.dumps(str(self.heading_toward_north)))
            out_file.close()

        self.measures.tofile(self.path + "/gns_data.geo_pose_conf_timetags.uint64.bin")
        self.timetags.tofile(self.path + "/gns_timetags.us.uint64.bin")

        with open('{0}/gns_0.json'.format(self.path), 'w+') as json_file:
            json.dump(self.gns.hal_gps.get_parameters(), json_file)

        print("{0} - GNS EXITING".format(time.time()), file=sys.stderr, flush=True)

        try:
            self.gns.shutdown()
        except:
            pass

        print("GNS Recorder: Acquired {0} number of GNS data".format(self.number_of_acquisitions))

    def warmup(self, n):
        print("GNS - Warming up...", end='')
        for i in range(n):
            self.gns.acquire()
            self.gns.reset()
        print(" - GNS Warmup Done.")

        self.starting_lat = np.float32(self.gns.get_starting_lat())
        self.starting_lon = np.float32(self.gns.get_starting_lon())

    def wait_fix(self):
        print("GNS - Waiting fix...", end='')
        while not self.gns.hal_gps.has_fix():
            pass
        print(" - GNS Wait Fix Done.")

        self.gns.reset()

        self.starting_lat = np.float32(self.gns.get_starting_lat())
        self.starting_lon = np.float32(self.gns.get_starting_lon())

