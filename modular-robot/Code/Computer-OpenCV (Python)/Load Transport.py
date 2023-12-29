# import pupil_apriltags as apriltag     # for windows
import numpy as np
import apriltag
import cv2
import math
import serial
import time

x = 0
y = 1

ser1 = serial.Serial('/dev/cu.usbserial-1420', 115200)
# ser2 = serial.Serial('/dev/cu.usbserial-1120', 115200)
# ser3 = serial.Serial('/dev/cu.usbserial-1130', 115200)

# Target coordinate of each of the modules (Usually one is used at a time)
tag1_target_coordinates = [1400, 900]
tag2_target_coordinates = [900, 480]
tag3_target_coordinates = [1150,580]
tag4_target_coordinates = [455, 345]

tag1_target_1 = [0,0]
tag1_target_2 = [0,0]
latest_tag1_cord = [0,0]

first_time_load = False

module_state = [0,1,1,1] # the index of each state corresponds with the tag_id, module_state[tag.tad_id], state 1,2,3,4



# 将目标点放进列表中 0-3
#index zero is only there so that the data can be accessed as all_target_coordinates[tag_id]
#(continued) instead of all_target_coordinates[tag_id-1]
all_target_coordinates = [[0,0],tag1_target_coordinates, tag2_target_coordinates, tag3_target_coordinates,
                          tag4_target_coordinates]


# The Coordinate plane max value is 1920,1080 which is the same as the camera resolution
#linux    640x480

# 根据坐标用反三角函数算夹脚 左上方的右边 机器人圆的一边是前面
# Calculates angle to the positive x axis using the coordinates of the top left, right corners of the april tag and middle
def get_angle(left_top, right_top, middle):
    top_midpoint = [(left_top[x] + right_top[x]) / 2, (left_top[y] + right_top[y]) / 2]
    theta = (math.atan2(middle[y] - top_midpoint[y], top_midpoint[x] - middle[x]))
    return theta

cap = cv2.VideoCapture(1)
at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11')) # april family type
# at_detector = apriltag.Detector(families='tag36h11 tag25h9')  #for windows

# distance needed to travel calculated through distance formula
def get_distance_to_target(current_coordinate, target_coordinate):
    dist = math.sqrt(
        (target_coordinate[x] - current_coordinate[x]) ** 2 + ((target_coordinate[y] - current_coordinate[y]) ** 2))
    return dist

# difference in angle that the robot is currently facing to where it needs to face
def to_target_angle(currentangle, current_coordinates, target_coordinates):
    large_angle = (
        math.atan2(current_coordinates[y] - target_coordinates[y], target_coordinates[x] - current_coordinates[x]))
    needed_angle = math.atan2(math.sin(large_angle - currentangle), math.cos(large_angle - currentangle))
    return needed_angle * 57.2957795

def to_horizontal_angle(currentangle, current_coordinates):
    large_angle = (
        math.atan2( current_coordinates[y] - 0 , 1 - current_coordinates[x] )
    )
    needed_angle = math.atan2( math.sin(large_angle - currentangle), math.cos( large_angle - currentangle ) )
    return needed_angle * 57.2957795

# Calculates the preliminary point that the robot first travels to before picking up the load
# This allows the robot to first adjust position before moving towards the load
def pre_point_calc(center_cord, current_angle, distance):
    pre_point_cord = [0, 0]
    pre_point_cord[x] = round(center_cord[x] - (distance * math.cos(current_angle)))
    pre_point_cord[y] = round(center_cord[y] + (distance * math.sin(current_angle)))
    return pre_point_cord


i = 0

tag1_end_flag = 0
tag2_end_flag = 0
tag3_end_flag = 0

while True:
    # captured camera frame
    ret, frame = cap.read()
    # 检测按键
    k = cv2.waitKey(1)
    if k == 27:  # press esc to quit
        break
    elif k == ord('s'):
        cv2.imwrite('E:/OpenCV_pic/' + str(i) + '.jpg', frame)
        i += 1
    # detects apriltag
    # frame = cv2.resize(frame, (1920, 1080))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray)

    for tag in tags:

        cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)  # left-top
        cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2)  # right-top
        cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2)  # right-bottom
        cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)  # left-bottom
        if(tag.tag_id < 10):
            cv2.circle(frame, tuple(all_target_coordinates[tag.tag_id]), 6, (0, 0, 255),4)  # destination point of robot

        # this sets the target coordinates for the robot go to as it goes to pick of the load
        if tag.tag_id == 11:
            if first_time_load==False:
                # tag1_target_1 is the preliminary point for the robot to go to before carrying (robot 1)
                tag1_target_1 = pre_point_calc(tag.center, get_angle(tag.corners[0], tag.corners[1], tag.center), 350)
                # tag1_target_2 is the point that the robot references as it goes towards the load
                tag1_target_2 = tag.center

                if module_state[tag.tag_id-10] >= 1:
                    first_time_load = True
            else:
                latest_tag1_cord = tag.center

            thepoint = pre_point_calc(tag.center, get_angle(tag.corners[0], tag.corners[1], tag.center), 350)
            cv2.circle(frame, tuple(thepoint), 5, (255, 0, 200), 5)
    # OpenCV interface view
    cv2.imshow('capture', frame)
    # print("tags: {}".format(tags))



    for tag in tags:
        #for each time it loops it sets the corresponding module to the corresponding carrying basket
        #the tag_id of the robots are less than 10, while the tag_id of the loads are great than 10
        if tag.tag_id < 10:
            if module_state[tag.tag_id] == 1:
                if get_distance_to_target(tag.center, tag1_target_1) > 20:
                    data = ""
                    data += str(tag.tag_id)
                    data += ","
                    data += str(1.1*round(get_distance_to_target(tag.center, tag1_target_1), 2))
                    data += ","
                    data += str(round(to_target_angle(get_angle(tag.corners[0], tag.corners[1], tag.center), tag.center,
                                                      tag1_target_1), 3))
                    data += ","
                    data += str(module_state[tag.tag_id])
                    data += "/"
                    ser1.write(data.encode())
                    print("current state: " + str(module_state[tag.tag_id]))
                else:
                    module_state[tag.tag_id] = 2


            if(module_state[tag.tag_id] == 2):
                if(to_target_angle(get_angle(tag.corners[0], tag.corners[1], tag.center),tag.center,tag1_target_2)>5):
                    data = ""
                    data += str(tag.tag_id)
                    data += ","
                    data += str(0)
                    data += ","
                    data += str(round(to_target_angle(get_angle(tag.corners[0], tag.corners[1], tag.center), tag.center,
                                                      tag1_target_2), 3))
                    data += ","
                    data += str(module_state[tag.tag_id])
                    data += "/"
                    ser1.write(data.encode())
                else:
                    module_state[tag.tag_id] = 3

            #goes to the load
            if(module_state[tag.tag_id] == 3):
                if(get_distance_to_target(latest_tag1_cord,tag1_target_2) < 40):
                    print(get_distance_to_target(latest_tag1_cord,tag1_target_2))
                    data = ""
                    data += str(tag.tag_id)
                    data += ","
                    #the 0.4 lowers the distance value so when the robot is approaching the load the motor output is lower
                    data += str(round(0.4*get_distance_to_target(tag.center, tag1_target_2), 2))
                    data += ","
                    data += str(round(to_target_angle(get_angle(tag.corners[0], tag.corners[1], tag.center), tag.center,
                                                      tag1_target_2), 3))
                    data += ","
                    data += str(module_state[tag.tag_id])
                    data += "/"
                    ser1.write(data.encode())
                else:
                    module_state[tag.tag_id] = 4

            if(module_state[tag.tag_id] == 4):
                if(get_distance_to_target(tag.center,all_target_coordinates[tag.tag_id]) > 20):
                    data = ""
                    data += str(tag.tag_id)
                    data += ","
                    data += str(round(get_distance_to_target(tag.center, all_target_coordinates[tag.tag_id]), 2))
                    data += ","
                    data += str(round(to_target_angle(get_angle(tag.corners[0], tag.corners[1], tag.center), tag.center,
                                                      all_target_coordinates[tag.tag_id]), 3))
                    data += ","
                    data += str(module_state[tag.tag_id])
                    data += "/"
                    ser1.write(data.encode())
                else:
                    module_state[tag.tag_id] = 5

            if(module_state[tag.tag_id] == 5):
                    data = ""
                    data += str(tag.tag_id)
                    data += ","
                    data += str(0.0)
                    data += ","
                    data += str(0.0)
                    data += ","
                    data += str(module_state[tag.tag_id])
                    data += "/"
                    ser1.write(data.encode())





        #
        # if tag1_end_flag and tag2_end_flag and tag3_end_flag:
        #     data = ""
        #     data += str(tag.tag_id)
        #     data += ",3000,"
        #     data += str(round(to_target_angle(get_angle(tag.corners[0], tag.corners[1], tag.center), tag.center,
        #                                       all_target_coordinates[tag.tag_id - 1]), 3))
        #     data += ","
        #     data += str(
        #         round(to_horizontal_angle(get_angle(tag.corners[0], tag.corners[1], tag.center), tag.center), 2))
        #     data += "/"
        #     if tag.tag_id == 1:
        #         ser1.write(data.encode())
        #     if tag.tag_id == 2:
        #         ser2.write(data.encode())
        #     if tag.tag_id == 3:
        #         ser3.write(data.encode())
        #
        # else:
        #     if tag.tag_id == 1:
        #         ser1.write(data.encode())
        #         if Range1 < 10 and Range2 < 100:
        #             tag1_end_flag = 1
        #         else:
        #             tag1_end_flag = 0
        #
        #     if tag.tag_id == 2:
        #         ser2.write(data.encode())
        #         if Range1 < 10 and Range2 < 100:
        #             tag2_end_flag = 1
        #         else:
        #             tag2_end_flag = 0
        #
        #     if tag.tag_id == 3:
        #         ser3.write(data.encode())
        #         if Range1 < 10 and Range2 < 100:
        #             tag3_end_flag = 1
        #         else:
        #             tag3_end_flag = 0

            # print(data)
            # print(frame.size)

cap.release()
cv2.destroyAllWindows()
