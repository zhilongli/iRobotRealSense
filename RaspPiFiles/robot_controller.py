import serial
import socket
import time
import math
import select
import _thread
import pyrealsense2 as rs
import apriltag
import os
import numpy as np
from PIL import Image
from threading import *


#Don't init camera on startup
tf = False

debug = False

#To handle the .so file not found error
def _get_demo_searchpath():
    
    return [
        os.path.join(os.path.dirname(__file__), '../build/lib'),
        os.path.join(os.getcwd(), '../build/lib')
    ]

def send_message(data, ser_port, serialLock):
    #first grab mutex lock
    serialLock.acquire()
    successful = False
    while not successful:
        try:
            ser_port.write(data)
            successful = True
        except select.error:
            pass

    #release the mutex lock
    serialLock.release()

def get_distance(depth_frame, num_points, height):
    ret = ""
    pix_per_deg = 12
    intervals = num_points-1
    pix_per_interval = int((640-4*num_points)/intervals)
    for j in range(num_points):
        start = j*pix_per_interval
        sum = 0
        for y in range(height*pix_per_deg-4, height*pix_per_deg):
           for k in range(start, start+4):
               sum +=depth_frame.get_distance(k, y)
        avg = sum/16
        print(start) 
        dist = depth_frame.get_distance(start, height*pix_per_deg)
        ret += str(avg)
        ret += " "
    return ret

def get_tag(color_frame, depth_frame, params):
    ret = ""
	#Initialize apriltage detector
 
    fov = 0.466 #in radians, half angle
    detector = apriltag.Detector(searchpath=_get_demo_searchpath())
    print ("Tag detector started!")
	#initialize image
    img = np.asanyarray(color_frame.get_data())
	#convert to grayscale
    pil_img = Image.fromarray(img)
    gray = np.array(pil_img.convert('L'))
    result = detector.detect(gray)
    num_detections = len(result)
    if num_detections==0:
        return "no tags detected"
 	
    print('Detected {} tags in {}\n'.format(
    num_detections, 'camera image'))
    print(result)
    i=0
    for detection in result:
        ret += str(i)
        ret += " "
        x, y = detection.center
        id = detection.tag_id
        ret += str(id)
        ret += " "
        dist_to_center = depth_frame.get_distance(int(x), int(y))
        print( 'Detection {} of {}:'.format(i+1, num_detections))
       # ret += 'Detection {} of {}:'.format(i+1, num_detections)
       # ret += str(dist_to_center)
       # ret += " "
        pose, e0, e1 = detector.detection_pose(detection, params, 0.165)
        z_dist = pose[2, 3]
        x_dist = pose[0, 3]-math.tan(fov)*z_dist
        yaw = math.atan2(pose[1, 0],pose[0, 0])
        ret += str(z_dist)
        ret += " "
        ret += str(x_dist)
        ret += " "
        ret += str(yaw)
        ret += " "

        print("pose of the tag is: ")
        print(pose)
        print(detection.tostring(indent=2))
		#ret += "Tag ID is: "
		#ret += str(ind)
       # ret.append(detection.tostring(indent=2))
        
        print('Distance to center of apriltag is: ')
       # ret += 'Distance to center of apriltag is: '
        print(dist_to_center)
        
        i+=1
    return ret
	#Compute and print the distance to center of apriltag


def main():

    #first establish connection with host computer
    TCP_IP = '199.168.1.100'
    TCP_PORT = 8865
    BUFFER_SIZE = 128
    debug = False
    serialLock = _thread.allocate_lock()

    #set the output serial property for create
    ser_port = serial.Serial("/dev/ttyUSB0", baudrate=57600, timeout=0.5)
    print ("Serial port set!")

    #start the socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    print("Waiting for connection...")

    conn, addr = s.accept()
    print ('Connected to address:')
    print (addr)

	# Configure depth and color streams
    print ("waiting for camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

	# Start streaming
    pipeline.start(config)
    print("Camera started and streaming!")
 	
    #align depth and color images
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        while 1:

            print ("Waiting for command...")
            dataAvail = select.select([conn],[],[],0.25)[0]
            while not dataAvail:
                bytesToRead = ser_port.inWaiting()
                if bytesToRead:
                    x = ser_port.read(bytesToRead)
                    print("data received from robot:")
                    print(x)
                    conn.send(x)
                dataAvail = select.select([conn],[],[],0.25)[0]
            data = (conn.recv(BUFFER_SIZE))
            print ("received data:")
            print (data)
            if data[:4] == b'dist':
                print("Waiting for frames from camera...")
                frames = pipeline.wait_for_frames()
                #align frames
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    print("waiting...")
                    continue
                print ("Got frames!")
	      
		
	#Get the distance readings from realsense camera
                data = data.decode('utf-8')
                chars = len(data)-4
                num_points = (data[4])
                height = (data[5:])
                print(height)
                dist_data = get_distance(depth_frame, int(num_points), int(height))
                print("SENDING TO COMPUTER")
                conn.send(str.encode(dist_data))
                print("DATA SENT TO COMPUTER")
            elif data == b'tag':
				#print("Waiting for frames from camera...")
                frames = pipeline.wait_for_frames()
                    #align frames
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                profile = pipeline.get_active_profile()
                rgb_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
                rgb_intrinsics = rgb_profile.get_intrinsics()
                print(rgb_intrinsics)
                params = [rgb_intrinsics.fx, rgb_intrinsics.fy, 0, 0]
                if not color_frame or not depth_frame:
                    print("waiting...")
                    continue
                print ("Got frames!")
                #get tag information from realsense camera
                tag_data = get_tag(color_frame, depth_frame, params)
                print("Sending data to computer...")
                conn.send(str.encode(tag_data))
                print("Data sent!")
            elif data == b'color':

                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                color_image = np.asanyarray(color_frame.get_data())
                pil_image = Image.fromarray(color_image)
                gray = np.array(pil_image.convert('L'))
                print(gray.shape)
                for i in range(480):
                    for j in range(640): 
                         conn.send(gray[i, j])

            elif data == b'depth':
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
               # depth_image = np.array(depth_frame.get_data())
               # print(depth_image.shape)
                to_send = ""
                for i in range(480):
                     
                     for j in range(640):
                         to_send += str(int(1000*depth_frame.get_distance(i, j)))
  #                       print(int(1000*depth_frame.get_distance(i, j)))
                         to_send +=" "
                
                conn.send(str.encode(to_send))

                print("Done sending depth image")
            else: 
		        #write data to port
                send_message(data, ser_port, serialLock)
                print ("data sent to the robot")
                time.sleep(1)
                
                bytesToRead = ser_port.inWaiting()
                x = ser_port.read(bytesToRead)
                print("data received from robot:")
                print(x)
                conn.send(x)

    finally:
        print("Something went wrong")
        #close TCP connection
        conn.shutdown(1)
        conn.close()
        #close the serial connection
        ser_port.close()
        pipeline.stop()

main()
