# -*- coding: utf-8 -*-
from bezier import bezier_curve,smoothing_base_bezier

import bezier
import matplotlib.pyplot as plt
import math
import numpy as np
import time
import argparse


global A_ErrorPast1 
global A_ErrorPast2 
global A_Mend 

A_ErrorPast1 = 0.0
A_ErrorPast2 = 0.0
A_Mend = 0.0

pi = 3.1415926536
DEG2RAD = 0.017453293

class Node:
  def __init__(self, x, y,):
    self.x = x  
    self.y = y  

global Start_Point
Start_Point = Node(0.0,0.0)

class PID(object):
  def __init__(self, p,i,d,mend,ep1,ep2):
    self.p = p
    self.i = i
    self.d = d
    self.mend = 0.0
    self.ep1 = 0.0
    self.ep2 = 0.0
  @staticmethod
  def AnglePID(Error,pid):
    global A_ErrorPast1 
    global A_ErrorPast2 
    global A_Mend 

    A = 0.0
    B = 0.0
    C = 0.0
    delta = 0.0

    A = (pid.p*(1+1/pid.i+pid.d))
    B = -pid.p*(1+2*pid.d)
    C = pid.p*pid.d
    
    delta = A*Error + B*A_ErrorPast1 + C*A_ErrorPast2
    A_Mend = A_Mend + delta
  
    A_ErrorPast2 = A_ErrorPast1
    A_ErrorPast1 = Error
    
    return A_Mend

  @staticmethod
  def PID_Clear():
    global A_ErrorPast1 
    global A_ErrorPast2 
    global A_Mend 
    global A_PID

    A_PID = PID(0.3,1000000,0,0,0,0)

    A_ErrorPast1 = 0.0
    A_ErrorPast2 = 0.0
    A_Mend = 0.0

global A_PID
A_PID = PID(1.0,1000000,0,0,0,0) #p=0.3

class Line_Point(object):
  def __init__(self, aim_position, speed_sta, speed_max, speed_end, rotation_dir):
    self.aim_position = self.myPoint(aim_position.x,aim_position.y)
    # self.aim_radian = aim_radian
    self.speed_sta = speed_sta
    self.speed_max = speed_max
    self.speed_end = speed_end
    self.rotation_dir = rotation_dir

  class myPoint(object):

    def __init__(self, x, y):
      self.x = x  # index of grid
      self.y = y  # index of grid
    @staticmethod
    def GetLineAngle(p1,p2):
      radian = math.atan2(p2.y - p1.y, p2.x - p1.x)
      return radian
    @staticmethod
    def GetLength(p1,p2):
      dx = (p1.x-p2.x)*(p1.x-p2.x)
      dy = (p1.y-p2.y)*(p1.y-p2.y)
      length = math.sqrt(dx+dy)
      return length;
    @staticmethod
    def GetFoot_P2L_PP(p,p1,p2):
      a=p2.y-p1.y
      b=p1.x-p2.x
      c=p2.x*p1.y-p1.x*p2.y

      foot = Node(0,0)

      foot.x=(b*b*p.x-a*b*p.y-a*c)/(a*a+b*b)
      foot.y=(a*a*p.y-a*b*p.x-b*c)/(a*a+b*b)
      return foot  

  def GoLine(self,cur_pos,cur_radian,A_PID):
    global Start_Point
    

    Sta_Point = Start_Point    #记录初始位置
    End_Point = self.aim_position
    # Aim_Radian = self.aim_radian
    

    Speed_Sta = self.speed_sta
    Speed_End = self.speed_end
    Speed_Max = self.speed_max
    
    Rotation_Dir = self.rotation_dir
    
    # Line_Angle = self.myPoint.GetLineAngle(Sta_Point,End_Point)  #求出该路径线段的倾角

    #print Line_Angle
    
    Line_Length = self.myPoint.GetLength(Sta_Point,End_Point)    #求出该路径线段的长度
    
    #print Line_Length

    Acc_Dis_Rate = (Speed_Max*Speed_Max - Speed_Sta*Speed_Sta)/(2*Speed_Max*Speed_Max - Speed_Sta*Speed_Sta - Speed_End*Speed_End)
    Dec_Dis_Rate = (Speed_Max*Speed_Max - Speed_End*Speed_End)/(2*Speed_Max*Speed_Max - Speed_Sta*Speed_Sta - Speed_End*Speed_End)
    

    Acc_Dis = Acc_Dis_Rate * Line_Length  #加速距离和减速距离
    Dec_Dis = Dec_Dis_Rate * Line_Length
    
    Acc = (Speed_Max * Speed_Max - Speed_Sta * Speed_Sta) / (2 * Acc_Dis)#加速度与减速度 v^2 = v0^2 + 2ax
    Dec = (Speed_Max * Speed_Max - Speed_End * Speed_End) / (2 * Dec_Dis)


    pos = self.myPoint.GetFoot_P2L_PP(cur_pos, Sta_Point, End_Point)
    # pos = cur_pos
    #根据理论坐标赋速度
    temp_dis = self.myPoint.GetLength(Sta_Point,pos)
    if temp_dis <= Acc_Dis:
      speed = math.sqrt(Speed_Sta * Speed_Sta + 2 * Acc * temp_dis)
      if speed > Speed_Max:
        speed = Speed_Max
    elif temp_dis >= Line_Length:
      speed = -math.sqrt(Speed_End * Speed_End + 2 * Dec * self.myPoint.GetLength(End_Point, pos))
      if speed < -Speed_Max:
        speed = -Speed_Max
    elif temp_dis >= (Line_Length - Dec_Dis):
      speed = math.sqrt(Speed_End * Speed_End + 2 * Dec * self.myPoint.GetLength(End_Point, pos))
      if speed > Speed_Max:
        speed = Speed_Max
    else:
      speed = Speed_Max
    

    #失败
    # cur_2_end_dis = self.myPoint.GetLength(cur_pos,End_Point)
    # if cur_2_end_dis < 50: #比较近
    #   Aim_Radian = self.myPoint.GetLineAngle(Sta_Point,End_Point)  #求出该路径线段的倾角，让车指向这个方向
    # else:
    #   Aim_Radian = self.myPoint.GetLineAngle(cur_pos,End_Point) #PID控制指向目标点
    #

    Aim_Radian = self.myPoint.GetLineAngle(cur_pos,End_Point) #PID控制指向目标点
    error_angle = Aim_Radian - cur_radian

    # print ('error_angle:{},Aim_Radian:{}'.format(error_angle,Aim_Radian))

    if math.fabs(math.fabs(error_angle) - pi) < 5 * DEG2RAD:
      if Rotation_Dir == 1:
        while error_angle < 0:
          error_angle = error_angle + 2 * pi
      elif Rotation_Dir == -1:
        while error_angle > 0:
          error_angle = error_angle - 2 * pi
    else:
      if error_angle > pi:
        error_angle = error_angle - 2 * pi
      if error_angle < -pi:
        error_angle = error_angle + 2 * pi
    
    #error_dis为垂直路径方向上偏离路径的距离，无误！
    #error_dis = (cur_pos.x - Sta_Point.x) * sin_temp -(cur_pos.y - Sta_Point.y) * cos_temp;
    
    #PID调整
    Vout_A = 3*PID.AnglePID(error_angle, A_PID)
    #Vout_D = PID.DistancePID(error_dis, D_PID)
    
    if Vout_A >= 0.5:  #300
      Vout_A = 0.5
    if Vout_A <= -0.5:
      Vout_A = -0.5
    
    return speed,Vout_A

  def Turn(self,aim_radian,cur_radian,A_PID):

    Rotation_Dir = self.rotation_dir
    Aim_Radian = aim_radian
    error_angle = Aim_Radian - cur_radian

    if math.fabs(math.fabs(error_angle) - pi) < 5 * DEG2RAD:
      if Rotation_Dir == 1:
        while error_angle < 0:
          error_angle = error_angle + 2 * pi
      elif Rotation_Dir == -1:
        while error_angle > 0:
          error_angle = error_angle - 2 * pi
    else:
      if error_angle > pi:
        error_angle = error_angle - 2 * pi
      if error_angle < -pi:
        error_angle = error_angle + 2 * pi

    Vout_A = 6*PID.AnglePID(error_angle, A_PID)
    
    if Vout_A >= 0.5:  #300
      Vout_A = 0.5
    if Vout_A <= -0.5:
      Vout_A = -0.5
    
    return Vout_A

def rount_filter(rx,ry,line_point):

  corner_point = []
  rest_point = []
  r_rest_point = []
  rount_radian = []

  print (len(rx))

  corner_point.append(0)
  

  for i in range(len(rx)):
    if i == len(rx) - 2:
      break
    if line_point.myPoint.GetLineAngle(Node(rx[i],ry[i]),Node(rx[i+1],ry[i+1])) != line_point.myPoint.GetLineAngle(Node(rx[i+1],ry[i+1]),Node(rx[i+2],ry[i+2])):
      corner_point.append(i+1)

  corner_point.append(len(rx)-1)

  print (corner_point)

  length = len(corner_point)

  for i in range(length): 
    if i > len(corner_point) - 1:
      break
    if i == len(corner_point) - 1:
      rest_point.append(corner_point[i])
      break

    if corner_point[i+1]-corner_point[i] == 3:  
      rest_point.append(corner_point[i]+1)
      corner_point.remove(corner_point[i+1])

    elif corner_point[i+1]-corner_point[i] == 2:
      rest_point.append(corner_point[i]+1)
      corner_point.remove(corner_point[i+1])

    elif corner_point[i+1]-corner_point[i] == 1:
      rest_point.append(corner_point[i])
      corner_point.remove(corner_point[i+1])
    else:
      rest_point.append(corner_point[i])

  print (rest_point)

  length = len(rest_point)

  for i in range(length): 
    if i > len(rest_point) - 1:
      break
    if i == len(rest_point) - 1:
      rest_point.append(rest_point[i])
      break

    # if rest_point[i+1]-rest_point[i] == 3:  
    #   r_rest_point.append(rest_point[i]+1)
    #   rest_point.remove(rest_point[i+1])    #放宽

    elif rest_point[i+1]-rest_point[i] == 2:
      r_rest_point.append(rest_point[i]+1)
      rest_point.remove(rest_point[i+1])

    elif rest_point[i+1]-rest_point[i] == 1:
      r_rest_point.append(rest_point[i])
      rest_point.remove(rest_point[i+1])
    else:
      r_rest_point.append(rest_point[i])

  if r_rest_point[len(r_rest_point)-1] != len(rx)-1:
    r_rest_point.append(len(rx)-1)
  if r_rest_point[0] != 0:
    if r_rest_point[0] < 4:
      rest_point.remove(rest_point[0])
      r_rest_point.insert(0,0)
    else:
      r_rest_point.insert(0,0)

  print (r_rest_point)
  tmp_x = []
  tmp_y = []

  for i in r_rest_point:
    tmp_x.append(rx[i])
    tmp_y.append(ry[i])


  x = np.array(tmp_x)
  y = np.array(tmp_y)

  plt.plot(x, y, 'ro')
  x_curve, y_curve = smoothing_base_bezier(x, y, k=0.3, closed=False)
  plt.plot(x_curve, y_curve, label='$k=0.3$')
  plt.axis('equal')
  plt.show()

  print (x_curve, y_curve)


  for i in range(len(x_curve)):
    if i == len(x_curve) - 1:
      break
    rount_radian.append(line_point.myPoint.GetLineAngle(Node(x_curve[i],y_curve[i]),Node(x_curve[i+1],y_curve[i+1])))

  print (rount_radian)

  bk_rount_radian = []
  rount_radian.reverse()

  for i in range(len(rount_radian)):
    if rount_radian[i] >= 0:
      bk_rount_radian.append(rount_radian[i] - pi)
    else:
      bk_rount_radian.append(rount_radian[i] + pi)

  print (bk_rount_radian)


  rount_radian.reverse()

  #return r_rest_point,rount_radian,bk_rount_radian
  return x_curve,y_curve,rount_radian,bk_rount_radian 

def To_Aim_Point(aim_position, line_point, end_radian, gazebo_sim):

  global Start_Point

  PID.PID_Clear()
  loc_b0_x = 0.0
  loc_b0_y = 0.0
  Cur_Radian = 0.0
  Speed_X = 0.0

  while True:

    
    pose = gazebo_sim.get_model_state().pose
    quat = pose.orientation
    
    from scipy.spatial.transform import Rotation as R
    r1 = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    Cur_Radian = r1.as_euler('xyz', degrees=False)[2]
    
    loc_b0_x = pose.position.x*1000  
    loc_b0_y = pose.position.y*1000
    Cur_Pos = Node(loc_b0_x,loc_b0_y)

    if line_point.myPoint.GetLength(Cur_Pos,aim_position) < 250:
      #action.sendCommand(vx=0, vy=0, vw=0)
      Start_Point = Node(loc_b0_x,loc_b0_y)
      break

    
    #print Cur_Pos.x,Cur_Pos.y
    Speed_X,Speed_Rotation = line_point.GoLine(Cur_Pos,Cur_Radian,A_PID)    #GoLine(self,cur_pos,cur_radian,A_PID):

    # action.sendCommand(vx=Speed_X, vy=0, vw=Speed_Rotation)
    gazebo_sim.pub_cmd_vel([Speed_X/1000, Speed_Rotation])
    #print (Speed_X,Speed_Rotation)
    # print (aim_position.x,aim_position.y)

    time.sleep(0.01)

  # if isToEnd == True:
  #   End_Radian = -3*pi/4 #line_point.myPoint.GetLineAngle(Start_Point,aim_position)   
  #有问题 还要单独写转角函数？
  #写了，真的解决了吗？
  #利用向右下，可不可以都调整为-135°？
  #                               
  #要不要指向下一个点？
  # elif isToEnd == False:
  #   End_Radian = pi/4

  End_Radian = end_radian

  # print (End_Radian)


  PID.PID_Clear()
  while True:

    # for robot_blue in vision_frame.robots_blue:
    #   #print('Robot Blue {} pos: {} {}'.format(robot_blue.robot_id, robot_blue.x,robot_blue.y))
    #   if robot_blue.robot_id == 0:
    #     Cur_Radian = robot_blue.orientation
    #     break
    pose = gazebo_sim.get_model_state().pose
    quat = pose.orientation
    
    from scipy.spatial.transform import Rotation as R
    r1 = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    Cur_Radian = r1.as_euler('xyz', degrees=False)[2]

    if math.fabs(End_Radian - Cur_Radian) < 12 * DEG2RAD:
      #action.sendCommand(vx=0, vy=0, vw=0)
      break
    if math.fabs(math.fabs(End_Radian) + math.fabs(Cur_Radian)- 2 * pi ) < 12 * DEG2RAD:   #-pi到pi有突变
      #action.sendCommand(vx=0, vy=0, vw=0)
      break

    Speed_Rotation = line_point.Turn(End_Radian,Cur_Radian,A_PID)
    # action.sendCommand(vx=0, vy=0, vw=Speed_Rotation)
    gazebo_sim.pub_cmd_vel([0, Speed_Rotation])

    time.sleep(0.02)
  print ('arrive:{},{}'.format(aim_position.x,aim_position.y))

if __name__ == '__main__':

  # robot_num = 8
  # loc_b_y = np.zeros(robot_num - 1,dtype=float) #不记录蓝色零号机器人
  # loc_b_x = np.zeros(robot_num - 1,dtype=float) 
  # loc_y_y = np.zeros(robot_num,dtype=float)
  # loc_y_x = np.zeros(robot_num,dtype=float)

  # vision = Vision()
  # action = Action()
  # debugger = Debugger()

  # vision_frame = vision.vision_frame
  # data, server = vision.sock.recvfrom(4096)
  # vision_frame.ParseFromString(data)

  # for robot_blue in vision_frame.robots_blue:
  #   print('Robot Blue {} pos: {} {}'.format(robot_blue.robot_id, robot_blue.x,robot_blue.y))
  #   if robot_blue.robot_id == 0:
  #     loc_b0_x = robot_blue.x
  #     loc_b0_y = robot_blue.y
  #     Cur_Radian = robot_blue.orientation
  #     continue
  #   loc_b_x[robot_blue.robot_id - 1] = robot_blue.x
  #   loc_b_y[robot_blue.robot_id - 1] = robot_blue.y
  # for robot_yellow in vision_frame.robots_yellow:
  #   print('Robot Yellow {} pos: {} {}'.format(robot_yellow.robot_id,robot_yellow.x, robot_yellow.y))
  #   loc_y_x[robot_yellow.robot_id] = robot_yellow.x
  #   loc_y_y[robot_yellow.robot_id] = robot_yellow.y

  # Start_Point = Node(loc_b0_x,loc_b0_y)
  # aim_position = Node(loc_b0_x+10,loc_b0_y+10) #调整姿态
  # line_point = Line_Point(aim_position,30.0,400.0,30.0,1)  

  # rx, ry = my_astar_main(loc_b_y,loc_b_x,loc_y_y,loc_y_x) #retrun rounte (x,y)  
  # debugger.draw_some_points(rx, ry)
  from map_reader import readMapObstacles
  from gazebo_simulation import GazeboSimulation
  from astar import AStarPlanner
  import rospy
  import rospkg
  INIT_POSITION = [-2, 3, 1.57]
  astar_resolution = 0.1 # too small 0.05
  moving_smooth = True
  show_animation = True
  parser = argparse.ArgumentParser(description = 'my BARN navigation challenge')
  parser.add_argument('--world_idx', type=int, default=1, help='world index')
  args = parser.parse_args()
  
  if args.world_idx < 200: 
    robot_size = 0.47 # too large for world 299 0.45
  else:
    robot_size = 0.47
  
  
  
  Start_Point = Node(INIT_POSITION[0]*1000,INIT_POSITION[1]*1000)
  aim_position = Node(INIT_POSITION[0]*1000+10,INIT_POSITION[1]*1000+10) #调整姿态
  line_point = Line_Point(aim_position, 30.0, 400.0, 30.0,1)  

  
  gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
  ox, oy = readMapObstacles(args.world_idx)
  
  sx = -2  # [m]
  sy = 3  # [m]
  gx = -2  # [m]
  gy = 13  # [m]   
  if show_animation:  # pragma: no cover
    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal") 
  is_found_goal = False
  replan = False
  while is_found_goal == False:
    if replan == True:
      show_animation = False
    a_star = AStarPlanner(ox, oy, astar_resolution, robot_size, show_animation)
    rx, ry, is_found_goal = a_star.planning(sx, sy, gx, gy)
    robot_size = robot_size - 0.01
    replan = True
    print(">>>>>>>>>>>>>>>>>>>>>>>>>Replan<<<<<<<<<<<<<<<<<<<<<<<<<<")
  # print(rx, ry)
  if show_animation:  
    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    # plt.show()

  rx.reverse()
  ry.reverse()

  print (rx,ry)

  if moving_smooth == False:
    x_curve = rx
    y_curve = ry
    rount_radian = []
    for i in range(len(x_curve)):
      if i == len(x_curve) - 1:
        break
      rount_radian.append(line_point.myPoint.GetLineAngle(Node(x_curve[i],y_curve[i]),Node(x_curve[i+1],y_curve[i+1])))
  else:
    x_curve,y_curve,rount_radian,bk_rount_radian = rount_filter(rx,ry,line_point)
    
    
    
  rospy.init_node('planner', anonymous=True)

  # sta_rad = line_point.myPoint.GetLineAngle( Start_Point , Node(-2000,13500))
  # print ("aim_angle::::::::::",sta_rad)
  # To_Aim_Point( aim_position,line_point, 0, gazebo_sim)
  
  # gazebo_sim.pub_cmd_vel([0, 0])
  
  # quit()

  # for x in range(1):

  for i in range(len(x_curve)):
    aim_position = Node(x_curve[i]*1000,y_curve[i]*1000)
    line_point = Line_Point(aim_position,300.0,600.0,100.0,1)   #450.0   50.0
    if i == len(x_curve) - 1:
      # To_Aim_Point(aim_position,line_point,bk_rount_radian[0], gazebo_sim)
      pass
    else:
      To_Aim_Point(aim_position,line_point,rount_radian[i], gazebo_sim)
  
  gazebo_sim.pub_cmd_vel([0, 0])

    # x_curve = x_curve[::-1] #reverse
    # y_curve = y_curve[::-1]


    # for i in range(len(x_curve)):
    #   aim_position = Node(x_curve[i]*1000,y_curve[i]*1000)
    #   line_point = Line_Point(aim_position,300.0,800.0,300.0,1)   #450.0   50.0
    #   if i == len(x_curve) - 1:
    #     To_Aim_Point(aim_position,line_point,rount_radian[0], gazebo_sim)
    #   else:
    #     To_Aim_Point(aim_position,line_point,bk_rount_radian[i], gazebo_sim)

    # x_curve = x_curve[::-1]
    # y_curve = y_curve[::-1]
    #250mm 12度 0.45 0.1 300 600 100只过不了150
    #250mm 12度 0.45 0.1 300 600 100只过不了150 -> 0.47 可过150 299有点final小问题

  # action.sendCommand(vx=0, vy=0, vw=0)#停下