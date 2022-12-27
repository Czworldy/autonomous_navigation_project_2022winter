from PIL import Image
import numpy as np
from astar import AStarPlanner, my_astar
import matplotlib.pyplot as plt
import argparse
def readMap2Np(num=1):
  mapfile = "./src/jackal_helper/worlds/BARN/map_files/map_pgm_" + str(num) + ".pgm"
  im = Image.open(mapfile)    # 读取文件
  map_np = np.array(im)
  return map_np


def readMapObstacles(world_idx=1):
  map_np = readMap2Np(world_idx)     # read map
  ox, oy = [], []
  
  transMatFromMap2World = 0.15*np.mat([[0, 1, -30], [-1, 0, 66], [0, 0, 1]])
  for x_idx in range(map_np.shape[0]):
    for y_idx in range(map_np.shape[1]):
      if map_np[x_idx][y_idx] == 0: # black
        ob_point_inMap = np.mat([[x_idx], [y_idx], [1]])
        ob_point_inWorld = np.matmul(transMatFromMap2World, ob_point_inMap)
        # print(ob_point_inMap)
        ox.append(ob_point_inWorld[0, 0])
        oy.append(ob_point_inWorld[1, 0])
        # print(ox[-1], oy[-1])
  
  ox.append(-4.5) # add boundary
  oy.append(13.5)
  return ox, oy
  
  
if __name__ == "__main__":
  show_animation = True
  parser = argparse.ArgumentParser(description = 'my BARN navigation challenge')
  parser.add_argument('--world_idx', type=int, default=1, help='world index')
  args = parser.parse_args()
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
  a_star = AStarPlanner(ox, oy, 0.1, 0.4, show_animation)
  rx, ry = a_star.planning(sx, sy, gx, gy)
  # print(rx, ry)
  if show_animation:  
    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()
    
    
    
