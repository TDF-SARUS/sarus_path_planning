OB = -1 # Obstacle value, don't change!
FU = 0 # Value of fuel on the fuel dist_map
NO = 0 # Area to be mapped value

# Conversion constants
START = u"drone"
FUEL = u"fuel"

# GeoJSON Keys
KEY = u"type"
COVERAGE = u"coverage"
OBSTACLE = u"obstacle"
AREAS = [COVERAGE, OBSTACLE]
POINTS = [START, FUEL]
FEATURES = [AREAS, POINTS]
POINT = u"layers/POINT"
POLYG = u"layers/POLYGON"

# Projection constants
EPSG = 4326
CRS = "EPSG:4326"

# Geometry
GEOM_POINT = u"Point"
GEOM_POLYG = u"Polygon"
