#!/usr/bin/python3

from datetime import datetime

from base64 import b64encode
from threading import Thread

from http.client import HTTPConnection
from http.client import IncompleteRead

''' This is to fix the IncompleteRead error
    http://bobrochel.blogspot.com/2010/11/bad-servers-chunked-encoding-and.html'''
import http.client

import requests
from requests.auth import HTTPBasicAuth



def NiceToICY(self):
    class InterceptedHTTPResponse():
        pass
    import io
    line = self.fp.readline().replace(b"ICY 200 OK\r\n", b"HTTP/1.0 200 OK\r\n")
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
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    def run(self):
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode((self.ntc.ntrip_user + ':' + self.ntc.ntrip_pass).encode()).decode("ascii")
            }
        connection = HTTPConnection(self.ntc.ntrip_server)
        connection.set_debuglevel(1)
        connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga, headers)
        response = connection.getresponse()
        if response.status != 200: raise Exception("Request failed")
        buf = ""
        restart_count = 0
        while not self.stop:
            ''' This now separates individual RTCM messages and publishes each one on the same topic '''
            data = response.read(1)
            if len(data) != 0:
                if data[0] == 211:                        # Byte 0 | RTCMv3 preamble
                    buf = []
                    buf.append(data[0])
                    data = response.read(2)
                    buf.append(data[0])
                    buf.append(data[1])
                    cnt = data[0] * 256 + data[1]         # Byte 1-2 | 6 bits padding (000000) + 10 bit byte count 
                    data = response.read(2)
                    buf.append(data[0])
                    buf.append(data[1])
                    typ = (data[0] * 256 + data[1]) >> 4  # Byte 3-4 | 12 bits message number
                    print(str(datetime.now()), "Size:", cnt, " Type:", typ)
                    cnt = cnt + 1
                    for x in range(cnt):
                        data = response.read(1)
                        buf.append(data[0])
                    #print(buf)
                    buf = []
                else:
                    raise Exception("Bad RTCMv3 frame")
            else:
                ''' If zero length data, close connection and reopen it '''
                restart_count = restart_count + 1
                print("Zero length ", restart_count)
                connection.close()
                connection = HTTPConnection(self.ntc.ntrip_server)
                connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga, headers)
                response = connection.getresponse()
                if response.status != 200: raise Exception("Request failed")
                buf = ""

        connection.close()

class ntripclient:
    def __init__(self):
        self.ntrip_server = "158.102.7.10:2101"
        self.ntrip_user = "roncapat"
        self.ntrip_pass = "roncapat"
        self.ntrip_stream = "RTK_MAC/MAX_RTCM3"
        self.nmea_gga = "$GPGGA,100120.611,4504.828,N,00736.709,E,1,12,1.0,0.0,M,0.0,M,,*61"

        self.connection = None
        self.connection = ntripconnect(self)
        self.connection.start()

    def run(self):
        while(True):
            pass
        if self.connection is not None:
            self.connection.stop = True

if __name__ == '__main__':
    c = ntripclient()
    c.run()
