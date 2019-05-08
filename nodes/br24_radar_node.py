#!/usr/bin/env python

import rospy
from marine_msgs.msg import RadarSectorStamped
from marine_msgs.msg import RadarScanline
import br24_radar.br24_radar
import datetime


def radar_listener():
    pub = rospy.Publisher('/radar',RadarSectorStamped,queue_size=10)
    rospy.init_node('br24_radar')
    
    radar = br24_radar.br24_radar.Radar()
    #radar.turnOn()
    #radar.setRange(2500)

    timestamp = datetime.datetime.utcfromtimestamp(rospy.Time.now().to_time()).isoformat()

    while not rospy.is_shutdown():
        try:
            radar_data = radar.getData()
        except socket.error:
            break

        if radar_data is not None:

            sector = RadarSectorStamped()
            sector.header.stamp = rospy.get_rostime()
            sector.header.frame_id = 'radar'

            for sl in radar_data['scanlines']:
                scanline = RadarScanline()
                scanline.angle = sl['angle']
                scanline.range = sl['range']
                scanline.intensities = sl['intensities']
                sector.sector.scanlines.append(scanline)
            pub.publish(sector)

if __name__ == '__main__':
    try:
        radar_listener()
    except rospy.ROSInterruptException:
        pass
