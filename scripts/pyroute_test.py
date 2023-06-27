

from pyroutelib3 import Router # Import the router
router = Router("car","/home/user/test_ws/src/gps_nav/scripts/IISc_Map.osm") # Initialise it
origin_lat  = 13.02435  #13.0270060 #origin_pose.pose.position.y
origin_long = 77.56332

start = router.findNode(origin_lat, origin_long) # Find start and end nodes


app_latitude  = 13.02596
app_longitude = 77.5639
end = router.findNode(app_latitude, app_longitude)

status, route = router.doRoute(start, end) # Find the route - a list of OSM nodes

if status == 'success':
    routeLatLons = list(map(router.nodeLatLon, route)) # Get actual route coordinates
    print("name,description,latitude,longitude")
    for k,point in enumerate(routeLatLons):
        print(k,",wp,",point[0],",",point[1])