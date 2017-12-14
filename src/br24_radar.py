#!/usr/bin/env python

import socket
import struct
import rospy
from marine_msgs.msg import RadarSector
from marine_msgs.msg import RadarScanline

# see paper:
# http://www.roboat.at/fileadmin/user_upload/_temp_/Roboat/Publications/DabrowskiEtAl2011.pdf
#
# make sure route is available: sudo route add -net 224.0.0.0 netmask 224.0.0.0 cw4_internal


def radar_listener():
    pub = rospy.Publisher('/radar',RadarSector,queue_size=10)
    rospy.init_node('br24_radar')
    
    multicast_group = '236.6.7.8'
    server_address = ('', 6678)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.bind(server_address)

    # Tell the operating system to add the socket to
    # the multicast group on all interfaces.
    group = socket.inet_aton(multicast_group)
    mreq = struct.pack('4sL', group, socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    while not rospy.is_shutdown():
        data, address = sock.recvfrom(100000)

        #print('received {} bytes from {}'.format(len(data), address))
        #print(data)

        ns,ss = struct.unpack('<BH', data[5:8])
        #print 'ns:',ns,'ss:',ss
        
        sector = RadarSector()

        for i in range(ns):
            cursor = 8+(i*(ss+24))
            l,st,rc,a,scale = struct.unpack('<BBH4xH2xH', data[cursor:cursor+14])
            #print i,'l:',l,'st:',st,'rc:',rc,'a:',a,'scale:',scale,
            angle = a*360.0/4096
            scale_m = scale*10.0/1.41421356237
            #scale_m = scale*1.41421356237/10.0
            values = struct.unpack('512B',data[cursor+24:cursor+24+512])
            #print angle, scale_m, values[0:30], scale_m*30/512.0
            scanline = RadarScanline()
            scanline.angle = angle
            scanline.range = scale_m
            scanline.intensities = data[cursor+24:cursor+24+512]
            sector.scanlines.append(scanline)
        pub.publish(sector)
        

if __name__ == '__main__':
    try:
        radar_listener()
    except rospy.ROSInterruptException:
        pass
