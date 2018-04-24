#!/usr/bin/env python

import br24_radar

r = br24_radar.Radar()

out = open('ranges.csv','w')

ranges = []

for i in range(0,400):
    ranges.append(i*.25)

for i in range(200,500):
    ranges.append(i*.5)

for i in range(250,500):
    ranges.append(i)

for i in range(500,1500,5):
    ranges.append(i)

for i in range(1500,25000,25):
    ranges.append(i)


for i in ranges:
    #print i
    r.setRange(i)
    sector = None
    for j in range(5):
        sector = r.getData()
    if sector is None:
        print None
    else:
        sl = sector['scanlines'][0]
        print ','.join((str(i),str(sl['range'])))
        out.write(','.join((str(i),str(sl['range']))))
        out.write('\n')
