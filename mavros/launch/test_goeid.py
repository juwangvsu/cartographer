from pygeodesy.ellipsoidalVincenty import LatLon
from pygeodesy.elevations import geoidHeight2
from pygeodesy.elevations import elevation2
# convert lat,lon to elevation or geoid height, these two function
# use web service and limit to u.s.a positions
#p = LatLon(37.4, -77)
#>>> p
#>>> p.elevation2()
#(elevation=25.330000000115923, data_source='3DEP 1/3 arc-second')
#>>> p.geoidHeight2()
#(height=-34.276, model_name='GEOID12B')
#>>>elevation2(37.4,-77)
#(elevation=25.33, data_source='3DEP 1/3 arc-second')
#>>> geoidHeight2(37.4,-77)
#(height=-34.276, model_name='GEOID12B')

from pygeodesy.geoids import GeoidG2012B
#this pkg use the geoid grid data
# download g2012bu7.bin from https://www.ngs.noaa.gov/GEOID/GEOID12B/GEOID12B_CONUS.shtml
gint=GeoidG2012B("/home/student/Downloads/g2012bu7.bin")
p = LatLon(31.4, -77)
print(gint(p))
p = LatLon(37.4, -77)
print(gint(p))

