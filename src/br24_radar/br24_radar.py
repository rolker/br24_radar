#!/usr/bin/env python

import socket
import struct
import datetime

# see paper:
# http://www.roboat.at/fileadmin/user_upload/_temp_/Roboat/Publications/DabrowskiEtAl2011.pdf
#
# make sure route is available: sudo route add -net 224.0.0.0 netmask 224.0.0.0 cw4_internal
#
# see also: https://github.com/keesverruijt/BR24radar_pi/blob/master/src/br24Receive.cpp

class Radar:
    def __init__(self):
        image_multicast_group = '236.6.7.8'
        image_server_address = ('', 6678)

        self.image_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.image_sock.bind(image_server_address)
        self.image_sock.settimeout(1)

        # Tell the operating system to add the socket to
        # the multicast group on all interfaces.
        group = socket.inet_aton(image_multicast_group)
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self.image_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        self.control_multicast_group = ('236.6.7.10',6680)
        control_server_address = ('', 6680)

        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.control_sock.bind(control_server_address)
        self.control_sock.settimeout(1)
        ttl = struct.pack('b', 2)
        self.control_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
        
        
        
    def getData(self):

        data = struct.pack('<BB',0xa0,0xc1)
        sent = self.control_sock.sendto(data,self.control_multicast_group)
        #print 'sent keep alive: ',sent

        try:
            data, address = self.image_sock.recvfrom(100000)
        except socket.error:
            return None

        #print('received {} bytes from {}'.format(len(data), address))
        #print(data)

        scanline_count,scanline_size = struct.unpack('<BH', data[5:8])
        #print 'ns:',ns,'ss:',ss
        
        sector = {'scanlines':[]}

        for i in range(scanline_count):
            cursor = 8+(i*(scanline_size+24))
            
            #common
            
            header_len,status,scan_number,mark,angle,heading = struct.unpack('<BBHIHH', data[cursor:cursor+12])

            scanline = {}
            
            if status == 2: #valid
                if mark == 0x00440d0e:
                    #br24 and 3G mode
                    scanline['mode'] = 'br24/3G'
                    scale, = struct.unpack('<I', data[cursor+12:cursor+16])
                    scanline['range'] = (scale & 0x00ffffff)*10/1.41421356237
                else:
                    #4g mode
                    scanline['mode'] = '4G'
                    large_range,small_range,rotation = struct.unpack('<6xh4xhH', data[cursor:cursor+16])
                    if large_range == 128:
                        if small_range == -1:
                            scanline['range'] = 0
                        else:
                            scanline['range'] = small_range/4
                    else:
                        scanline['range'] = large_range*64
                    
                scanline['angle'] = angle*360.0/4096
                scanline['intensities'] = []
                for d in data[cursor+24:cursor+24+scanline_size]:
                    low = ord(d)&0x0f;
                    high = (ord(d)&0xf0)>>4;
                    scanline['intensities'].append(low)
                    scanline['intensities'].append(high)
                sector['scanlines'].append(scanline)
        return sector

    def turnOn(self):
        data = struct.pack('<BBB',0,0xc1,1)
        self.control_sock.sendto(data,self.control_multicast_group)
        data = struct.pack('<BBB',1,0xc1,1)
        self.control_sock.sendto(data,self.control_multicast_group)

    def turnOff(self):
        data = struct.pack('<BBB',0,0xc1,0)
        self.control_sock.sendto(data,self.control_multicast_group)
        data = struct.pack('<BBB',1,0xc1,0)
        self.control_sock.sendto(data,self.control_multicast_group)
        
    def setRange(self, range_meters):
        range_dm = int(range_meters*10)
        data = struct.pack('<BBI',3,0xc1,range_dm)
        self.control_sock.sendto(data,self.control_multicast_group)

    def scanSpeedHigh(self):
        data = struct.pack('<BBB',0x0f,0xc1,1)
        self.control_sock.sendto(data,self.control_multicast_group)

    def scanSpeedLow(self):
        data = struct.pack('<BBB',0x0f,0xc1,0)
        self.control_sock.sendto(data,self.control_multicast_group)


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        range_m = float(sys.argv[1])
    else:
        range_m = 50
    r = Radar()
    r.turnOn()
    r.setRange(range_m)
    r.scanSpeedHigh()
    while True:
        sector = r.getData()
        if sector is not None:
            for s in sector['scanlines']:
                print s,
                break
            print
        else:
            print sector
