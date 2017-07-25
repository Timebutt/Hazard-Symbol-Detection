# -*- coding: latin-1 -*-

import json
import urllib
import math
import time
from fastkml import kml
import pandas as pd
from bisect import bisect_left
import json
import os.path
#import msvcrt
import threading

import tornado.httpserver
import tornado.websocket
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

flight_number = 6
print(bcolors.WARNING + "LOADING" + bcolors.ENDC + " telemetry data for " + bcolors.OKGREEN + "testvlucht" + str(flight_number) + ".mp4" + bcolors.ENDC)

# Read the telemetry data
telemetry_buffer = pd.read_csv("testvlucht" + str(flight_number) + ".csv", sep=";")

def findClosest(myList, myNumber):
    pos = bisect_left(myList, myNumber)
    if pos == 0:
        return myList[0]
    if pos == len(myList):
        return myList[-1]
    before = myList[pos - 1]
    after = myList[pos]

    return pos

def convert_to_ms(x):
    timeString = x['Time [hh:mm:ss]'].split(":")
    return (int(timeString[0]) * 60 * 60 + int(timeString[1]) * 60 + int(timeString[2])) * 1000 + x['Time [milliSec]']

def add_heading(x):
    return math.degrees(float(x['Angle psi / Yaw'].replace(',', '.'))) % 360

# Process the telemetry data
telemetry_buffer['ms'] = telemetry_buffer.apply(convert_to_ms, axis=1)
telemetry_buffer['Drone GPS Longitude'] = telemetry_buffer['Drone GPS Longitude'].apply(lambda x: float(x.replace(',', '.')))
telemetry_buffer['Drone GPS Latitude'] = telemetry_buffer['Drone GPS Latitude'].apply(lambda x: float(x.replace(',', '.')))
#telemetry_buffer['Controller GPS Longitude'] = telemetry_buffer['Controller GPS Longitude'].apply(lambda x: float(x.replace(',', '.')))
#telemetry_buffer['Controller GPS Latitude'] = telemetry_buffer['Controller GPS Latitude'].apply(lambda x: float(x.replace(',', '.')))
telemetry_buffer['Altitude in meters'] = telemetry_buffer['Altitude in meters'].apply(lambda x: float(x.replace(',', '.')))
telemetry_buffer['heading'] = telemetry_buffer.apply(add_heading, axis=1)

# Configuration variables
max_time_delta = 150
max_buffer_time = 100000000000000000
min_positive_detections = 1
detection_probability_threshold = 0.40
R = 6371000
video_fps = 30

# Detection coordinates
controller_latitude = 51.01148
controller_longitude = 3.710145

#controller_latitude = 51.011462
#controller_longitude = 3.710109

#controller_latitude = 51.011451
#controller_longitude =  3.710096

# Detection coordinates (testvlucht 2 tem 4)
#controller_latitude = 51.011384
#controller_longitude = 3.712839

# Controller Coordinates (as determined with Android app - 3 minute interval)
controller_latitude = 51.011461
controller_longitude = 3.710089

# Read the currently available detections
detections_buffer = []
'''with urllib.request.urlopen("http://localhost/api.php") as url:
    data = json.loads(url.read().decode())
    #telemetry_buffer.extend(data['telemetry'])
    detections_buffer.extend(data['detections'])'''

# Camera configuration
drone_type = "Parrot-Bebop"
#object_width = 0.141421 # 10cm NFPA diamant
object_width = 0.282843 # 20cm NFPA diamant

if drone_type == "DJI-M100":
    horizontal_fov = 81.3
    image_width = 1080
    focal_length_processing = 400

elif drone_type == "Parrot-Bebop":
    horizontal_fov = 78
    vertical_fov = 49
    focal_length = 1 / 2.2
    image_width = 1080
    #focal_length_processing = 1220
    focal_length_processing = 2000

summation = { 'longitude': 0, 'latitude': 0 }
results = []

def processing():

    start_time = time.time()
    detection_results = {}
    detections_buffer = []
    counter = 0
    global synced
    global timeReference

    # GET the telemetry and detections data
    with urllib.request.urlopen("http://localhost/api.php") as url:
        data = json.loads(url.read().decode())
        #telemetry_buffer.extend(data['telemetry'])
        detections_buffer.extend(data['detections'])

    # Iterate over the detections buffer and match with the closest telemetry
    for detection in detections_buffer:
        time_delta = 10000000
        telemetry_item = {}

        # Better method: use the frame number to build up the timestamp
        timestamp_detection = int(detection['frame']) / video_fps * 1000

        # Sync telemetry and video
        timeReference = timestamp_detection
        if synced == False:
            synced = True
            print("Synced!")

        # Find the closest matching telemetry data point
        telemetry_position = findClosest(telemetry_buffer['ms'], timestamp_detection)
        telemetry = telemetry_buffer.iloc[telemetry_position, :]

        # We have a match, process the matched telemetry/detection pair
        if abs(telemetry['ms'] - timestamp_detection) < max_time_delta:

            # If there is no valid drone GPS single, there is no point in further processing
            if telemetry['Drone GPS Longitude'] == 500 or telemetry['Drone GPS Latitude'] == 500:
                continue

            # Ignore telemetry entries without corresponding heading
            if telemetry['heading'] == -1000:
                continue

            # Detection probability must be above defined threshold
            if float(detection['prob']) < detection_probability_threshold:
                continue

            # This is correct, since detection['loc_x'] has the center of the detected object (corrected when data is uploaded to server)
            correction_angle = horizontal_fov * (float(detection['loc_x']) - 0.5) / 2
            angle = telemetry['heading'] + correction_angle
            #measured_distance = 2 * object_width * focal_length_processing / (float(detection['width']) + float(detection['height']))
            measured_distance = object_width * focal_length_processing / max([float(detection['width']), float(detection['height'])])

            # TO-DO: remove this once we are sure telemetry data and video feed are synced
            # doesn't actually have to be removed, as this should never happen in case of a synced feed
            if measured_distance < telemetry['Altitude in meters']:
                continue
            ground_distance = math.sqrt(measured_distance**2 - telemetry['Altitude in meters']**2)*1.3

            #print("Measured distance is: " + str(measured_distance) + " | Ground distance is: " + str(ground_distance) + " | Afstand tussen GPS coördinaten: " + str(distance_between_gps_coordinates(telemetry['Drone GPS Latitude'], telemetry['Drone GPS Longitude'], controller_latitude, controller_longitude)))

            # Save to be sent to the MYSQL database
            drone_lat = telemetry['Drone GPS Latitude']
            drone_lon = telemetry['Drone GPS Longitude']
            heading = telemetry['heading']

            #print("Measured distance" , measured_distance)
            #print("Ground distance" , ground_distance)

            # Distances in m
            dx = ground_distance * math.sin(angle*math.pi/180)
            dy = ground_distance * math.cos(angle*math.pi/180)

            # Calculate result (simplified results)
            lat = telemetry['Drone GPS Latitude'] + (180/math.pi)*(dy/6378137)
            lon = telemetry['Drone GPS Longitude'] + (180/math.pi)*(dx/6378137)/math.cos(telemetry['Drone GPS Latitude']*math.pi/180)

            # Calculate detection coordinates (accurate calculation)
            #delta = ground_distance / R
            #lat = math.asin(math.sin(telemetry['Drone GPS Latitude']*math.pi/180)*math.cos(delta) +  math.cos(telemetry['Drone GPS Latitude']*math.pi/180)*math.sin(delta)*math.cos(angle*math.pi/180))
            #lon = telemetry['Drone GPS Longitude'] + math.atan2(math.sin(angle*math.pi/180)*math.sin(delta)*math.cos(telemetry['Drone GPS Latitude']*math.pi/180), math.cos(delta) - math.sin(telemetry['Drone GPS Latitude']*math.pi/180)*math.sin(lat))
            #lat = lat * 180 / math.pi

            # Output the results
            if not detection['id'] in detection_results:
                detection_results[detection['id']]  = [(lat, lon)]
            else:
                detection_results[detection['id']].append((lat, lon))

            # Remove the detection entry
            #del detections_buffer[counter_detections]

        #else:
    #        counter_detections = counter_detections + 1

    # Average the position detections made
    for key in detection_results:
        value = detection_results[key]
        lat_temp = 0
        lon_temp = 0

        # Sum all the intermediary results
        for u in value:
            lat_temp = lat_temp + u[0]
            lon_temp = lon_temp + u[1]

        # Get the average
        lat_temp = lat_temp / len(value)
        lon_temp = lon_temp / len(value)

        # Ignore 'false' detections (ghost detections)
        if len(value) >= min_positive_detections:
            if counter in results:
                results[counter] = {'latitude': lat_temp, 'longitude': lon_temp, 'nr_detections': len(value)}
            else:
                results.append({'latitude': lat_temp, 'longitude': lon_temp, 'nr_detections': len(value)})

    # Average over long term detections
    if len(results) > 1:
        lat_temp = 0
        lon_temp = 0
        for u in results:
            lat_temp = lat_temp + u['latitude']
            lon_temp = lon_temp + u['longitude']
        lat_temp = lat_temp / len(results)
        lon_temp = lon_temp / len(results)

        print("Detection coordinates" + bcolors.WARNING + str((lat_temp, lon_temp)) + bcolors.ENDC)
        print(("Distance between average detection and controller is " + bcolors.OKGREEN + "{:.4f}m" + bcolors.ENDC).format(distance_between_gps_coordinates(lat_temp, lon_temp, controller_latitude, controller_longitude)))

    # Save data to file for visualization
    if len(results) > 0 and 'drone_lat' in locals():

        save_path = "C:/wamp64/www/3DSafeGuard"
        name_of_file = "detections.json"
        completeName = os.path.join(save_path, name_of_file)

        file_object  = open(completeName, "w")
        object = {'drone_lat': drone_lat, 'drone_lon': drone_lon, 'heading': int(heading), 'detection_lat': results[len(results)-1]['latitude'], 'detection_lon': results[len(results)-1]['longitude'], 'average_lat': lat_temp, 'average_lon': lon_temp}
        file_object.write(json.dumps(object, indent=4, sort_keys=True))
        file_object.close()

    # Just save the drone position
    '''elif timeReference > -1000:

        telemetry_position = findClosest(telemetry_buffer['ms'], timeReference)
        telemetry = telemetry_buffer.iloc[telemetry_position, :]

        save_path = "C:/wamp64/www/3DSafeGuard"
        name_of_file = "detections.json"
        completeName = os.path.join(save_path, name_of_file)

        file_object  = open(completeName, "w")
        object = {'drone_lat': float(telemetry['Drone GPS Latitude']), 'drone_lon': float(telemetry['Drone GPS Longitude']), 'heading': float(telemetry['heading']), 'detection_lat': 0, 'detection_lon': 0, 'average_lat': 0, 'average_lon': 0}
        file_object.write(json.dumps(object, indent=4, sort_keys=True))
        file_object.close()'''

    counter = counter + 1
    print("--- %.4f seconds ---" % (time.time() - start_time))

# Calculate the distance between two coordinates
def distance_between_gps_coordinates(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2-lat1)
    delta_lambda = math.radians(lon2-lon1)
    a = math.sin(delta_phi/2) * math.sin(delta_phi/2) + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2) * math.sin(delta_lambda/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

# Run the processing method once every second
start_time = time.time()
synced = False
timeReference = -1000

# Main loop
execution_interval = 500 # interval in ms
telemetry_interval = 100 # interval in ms
def execute():

    global synced, timeReference

    # Execute main processing
    processing()
    time.sleep(execution_interval / 1000)

    # This part is to sync manually (on keypress)
    # if msvcrt.kbhit() and synced == False:
    #     print (bcolors.OKGREEN + "Synced!" + bcolors.ENDC)
    #     timeReference = 0
    #     synced = True

    # elif synced == True:
    #     timeReference = timeReference + execution_interval
    #
    #     telemetry_position = findClosest(telemetry_buffer['ms'], timeReference)
    #     telemetry = telemetry_buffer.iloc[telemetry_position, :]
    #
    #     save_path = "C:/wamp64/www/3DSafeGuard"
    #     name_of_file = "drone.json"
    #     completeName = os.path.join(save_path, name_of_file)
    #
    #     file_object  = open(completeName, "w")
    #     object = {'drone_lat': telemetry['Drone GPS Latitude'], 'drone_lon': telemetry['Drone GPS Longitude'], 'heading': telemetry['heading']}
    #     file_object.write(json.dumps(object, indent=4, sort_keys=True))
    #     file_object.close()

    # Run again
    execute()

# Object that will containt the current drone telemetry data
drone_telemetry = {}
drone_telemetry['latitude'] = 0
drone_telemetry['longitude'] = 0

class WSHandler(tornado.websocket.WebSocketHandler):

    def check_origin(self, origin):
        return True

    def open(self):
        global execution_interval
        print("Connection opened from " + bcolors.OKGREEN + self.request.remote_ip + bcolors.ENDC)

        self.callback = PeriodicCallback(self.send_telemetry, telemetry_interval)
        self.callback.start()

    def send_telemetry(self):

        global synced, timeReference, telemetry_buffer, execution_interval
        if synced == True:

            timeReference = timeReference + telemetry_interval
            telemetry_position = findClosest(telemetry_buffer['ms'], timeReference)
            telemetry = telemetry_buffer.iloc[telemetry_position, :]
            telemetry_JSON = {'latitude': telemetry['Drone GPS Latitude'], 'longitude': telemetry['Drone GPS Longitude'], 'heading': telemetry['heading']}

            self.write_message(json.dumps(telemetry_JSON))

    def on_message(self, message):
        pass

    def on_close(self):
        print("Connection closed from " + bcolors.OKGREEN + self.request.remote_ip + bcolors.ENDC)
        self.callback.stop()

application = tornado.web.Application([(r'/', WSHandler),])
def startServer():
    print(bcolors.WARNING + "STARTING" +bcolors.ENDC + " WebSocket server at port " + bcolors.OKGREEN + "9002" + bcolors.ENDC)
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9002)
    tornado.ioloop.IOLoop.instance().start()

# Create main execution thread
t1=threading.Thread(target=execute)
t1.daemon = True
t1.start()

# Create WebSocket server thread
t2=threading.Thread(target=startServer)
t2.daemon = True
t2.start()

# Keep the script running while not using CPU power
while(1):
    time.sleep(10)
