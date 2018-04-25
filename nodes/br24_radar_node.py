#!/usr/bin/env python

import rospy
import rosbag
from marine_msgs.msg import RadarSector
from marine_msgs.msg import RadarScanline
import br24_radar.br24_radar
import datetime


def radar_listener():
    pub = rospy.Publisher('/radar',RadarSector,queue_size=10)
    rospy.init_node('br24_radar')
    
    radar = br24_radar.br24_radar.Radar()
    radar.turnOn()
    radar.setRange(2500)

    timestamp = datetime.datetime.utcfromtimestamp(rospy.Time.now().to_time()).isoformat()
    bag = rosbag.Bag('nodes/br24_radar_'+('-'.join(timestamp.split(':')))+'.bag', 'w')

    while not rospy.is_shutdown():
        try:
            radar_data = radar.getData()
        except socket.error:
            break

        if radar_data is not None:

            sector = RadarSector()

            for sl in radar_data['scanlines']:
                scanline = RadarScanline()
                scanline.angle = sl['angle']
                scanline.range = sl['range']
                scanline.intensities = sl['intensities']
                sector.scanlines.append(scanline)
            pub.publish(sector)
            bag.write('/radar',sector)
    bag.close()

if __name__ == '__main__':
    try:
        radar_listener()
    except rospy.ROSInterruptException:
        pass
