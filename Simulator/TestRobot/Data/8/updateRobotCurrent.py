# -*- coding: utf-8 -*-
"""
Created on Mon Jul  6 16:32:27 2020

@author: 218019067
"""
# read robot
with open("Robot_Init_Position.txt",'r') as file:
    lines = file.readlines()
    robot_number = lines[0]
    print(robot_number)
    # read data
    robot_current = [[] for i in range(len(lines)-1)]
    for i in range(len(robot_current)):
        robot_current[i][:] = (item for item in lines[i+1].strip().split(','))
        print(robot_current[i])
        
# read task
with open("Task.txt",'r') as file:
    lines = file.readlines()
    # read data
    robot_target = [[] for i in range(len(lines)-1)]
    for i in range(len(robot_target)):
        robot_target[i][:] = (item for item in lines[i+1].strip().split(','))
        print(robot_target[i])
        
# write to robot
with open("Robot_Current_Position.txt",'w+') as file:
    file.truncate()
    file.write(robot_number)

    for i in range(int(robot_number)):
        w = str(robot_current[i]).strip('[').strip(']').replace(' ','').replace('\'','') + \
               ',' + str(robot_target[i][1:]).strip('[').strip(']').replace(' ','').replace('\'','') + '\n'
        file.write(w)
