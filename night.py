import cv2
from pymavlink import mavutil
from time import sleep
import time
import sys
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math 
from sensor_msgs.msg import Image
from cv2 import aruco

class drone:
    def __init__(self):
        rospy.init_node("CV")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera_head/image_raw",Image,self.callback_rgb)
        self.imageshow_rgb=np.zeros((640,480,3), dtype = np.uint8)

    def callback_rgb(self,data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.imageshow_rgb=img

    def Image_shower(self):
        cv2.imshow("rgb", self.imageshow_rgb)
        cv2.waitKey(1)
     

if __name__ == '__main__':
    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')


    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
        (the_connection.target_system, the_connection.target_component))


    mode = 'GUIDED'
    mode_id = the_connection.mode_mapping()[mode]
    the_connection.mav.set_mode_send(the_connection.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    # arm
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

      

    # takeoff

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 2)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    
    while msg.relative_alt<3:
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print(msg.relative_alt)
        
    sleep(10)
    # the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
    #                                 mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 30, 5, 1, 0, 0, 0, 0)

    
    drone = drone()
    r = rospy.Rate(100)
    flag=0

    calib_data_path = "/home/drone_singh/Downloads/MultiMatrix.npz"
    calib_data = np.load(calib_data_path)
    cam_mat = calib_data["camMatrix"] 
    dist_coef = calib_data["distCoef"]
    r_vectors = calib_data["rVector"]
    t_vectors = calib_data["tVector"]
    pi = math.pi
    MARKER_SIZE = 2.50  #cm
    marker_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    param_markers = aruco.DetectorParameters_create()
    # param_markers.adaptiveThreshWinSizeMin = 3       
    # param_markers.adaptiveThreshWinSizeMax = 5


    dt=0.01
    temp_x=0
    temp_y=0
    temp_z=0
    start_time= time.time()
    current_time= time.time()
    xprev=0
    yprev=0
    time_flag=1
    first=0
    z_min=500
    while not rospy.is_shutdown() and not flag==1:
        try:
            key = cv2.aruco.DICT_ARUCO_ORIGINAL
            frame= drone.imageshow_rgb
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, reject = aruco.detectMarkers(
                gray_frame, marker_dict, parameters=param_markers
            )

            x = 0
            y = 0
            z = 0
            if marker_corners:
                rVec, tVec,_ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat, dist_coef)
                (rVec-tVec).any()
                total_markers = range(0, marker_IDs.size)
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    cv2.polylines(
                        frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                    )
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    top_left = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    bottom_left = corners[3].ravel()
                    
                    #Draw the pose 
                    poit = cv2.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i],tVec[i], 6, 4)

                    cv2.putText(
                        frame,
                        str("(x:" + str(round(tVec[i][0][0],3)) + ",y:" + str(round(tVec[i][0][1],3))+ ",z:" + str(round(tVec[i][0][2],3))+")" ),
                        (top_right[0],top_right[1]),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.3,
                        (26, 100, 120),
                        2
                    )

                    cv2.putText(
                        frame,
                        str("(Roll:" + str(abs(round((rVec[i][0][0])*180/pi,1))) + " deg,Pitch:" + str(abs(round((rVec[i][0][2])*180/pi,1))) + "deg,Yaw:" +str(abs(round((rVec[i][0][1])*180/pi,1)))+ "deg)"),
                        (0,400),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.3,
                        (26, 100, 120),
                        2
                    )
                    
                    x = round(tVec[i][0][0],3)
                    y = round(tVec[i][0][1],3)
                    z = round(tVec[i][0][2],3)
            # print("x =", x," ", "y =", y)
            try:
                if (marker_IDs==None):
                    out = True
                else:
                    out= False
            except:
                out = True
                pass
            # print("out = ",out)
            if not out:
                temp_x=x
                temp_y=y
                temp_z=z
            else :
                x=temp_x
                y=temp_y
                z=temp_z

            # velocity calculation 
            k = 0.2
            if abs(x)<2 and abs(y)<2:
                k=0.015


            dely= y
            delx= x
            if delx == 0 :
                theta = math.pi/4.0
            else:
                theta = math.atan2(abs(y),abs(x))

            if dely>=0 and delx>=0:  
                vx = -k*(math.sin(theta))
                vy = k*(math.cos(theta))

            elif dely<=0 and delx<=0: 
                vx = k*(math.sin(theta))
                vy =-k*(math.cos(theta))

            elif dely>0 and delx<0:
                vx = -k*(math.sin(theta))
                vy = -k*(math.cos(theta)) 
                 
            elif dely<0 and delx>0: 
                vx = k*(math.sin(theta))
                vy = k*(math.cos(theta))

            if x==0 and y==0:
                vx=0
                vy=0
           

            # m=(vx**2 + vy**2)**0.5
            # vx=vx/m
            # vy=vy/m
            
            
            current_time= time.time()
            time_elasped=current_time-start_time
            
            if time_elasped >4 or time_flag==1:
                print("entered")

                time_flag=0
                if abs(x- xprev) <3 and abs(y-yprev)<3:
                    print("absx: ",abs(x)," absy: ",abs(y)," absz: ",abs(z))
                    #landing sequence
                    if flag ==2 : 
                        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                            the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111000111), 0, 0, 0, vx, vy, 0.05, 0, 0, 0, 0, 0))
                        if z < z_min:
                            flag=1
                    #goal reached
                    elif (abs(x) < 2 and abs(y)< 2):
                        if first!=0:
                            print("entereeeeeeeeeeeed")
                            flag = 2
                        else:
                            first=1
                    #tracking at constant altitude
                    else:
                        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111000111), 0, 0, 0, vx, vy, 0, 0, 0, 0, 0, 0))
                xprev=x
                yprev=y
                start_time=  time.time()

            # print("later x = ", x," ", "later y = ", y)
            # print("vx = ", vx," ", "vy = ",vy)
            
            # check if threshhold reached
            
            

            # visualization
            cv2.imshow("Aruco detected", frame)
            
            if cv2.waitKey(1) & 0xFF ==ord('q'):  
                break


            # drone.Image_shower()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
    
    # landing

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)





