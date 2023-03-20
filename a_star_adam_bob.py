import numpy as np
import matplotlib.pyplot as plt
import time
import sys
from queue import PriorityQueue
import cv2 as cv

#defining each possible move
def moveForward(l, state):
    if state[2] >= 360:
        state[2] = state[2] - 360
    theta_next = state[2]
    x = np.cos(np.radians(theta_next))*l
    y = np.sin(np.radians(theta_next))*l
    next_loc = [state[0]+x,state[1]+y,theta_next]
    return next_loc

def moveLeft(l, state):
    if state[2] >= 360:
        state[2] = state[2] - 360
    theta_next = state[2] + 30
    x = np.cos(np.radians(theta_next))*l
    y = np.sin(np.radians(theta_next))*l    
    next_loc = [state[0]+x,state[1]+y,theta_next]
    return next_loc

def moveFullLeft(l, state):
    if state[2] >= 360:
        state[2] = state[2] - 360
    theta_next = state[2] + 60
    x = np.cos(np.radians(theta_next))*l
    y = np.sin(np.radians(theta_next))*l    
    next_loc = [state[0]+x,state[1]+y,theta_next]
    return next_loc

def moveRight(l, state):
    if state[2] >= 360:
        state[2] = state[2] + 360
    theta_next = state[2] - 30
    x = np.cos(np.radians(theta_next))*l
    y = np.sin(np.radians(theta_next))*l    
    next_loc = [state[0]+x,state[1]+y,theta_next]
    return next_loc

def moveFullRight(l, state):
    if state[2] >= 360:
        state[2] = state[2] + 360
    theta_next = state[2] - 60
    x = np.cos(np.radians(theta_next))*l
    y = np.sin(np.radians(theta_next))*l    
    next_loc = [state[0]+x,state[1]+y,theta_next]
    return next_loc

#defining moveset for each node
def moveset(map, state, l):
    next_node = []
    next_node.append(moveFullLeft(l,state))
    next_node.append(moveLeft(l,state))
    next_node.append(moveForward(l,state))
    next_node.append(moveRight(l,state))
    next_node.append(moveFullRight(l,state))
    temp_node = next_node.copy()
    for i in range(0, len(next_node)):
        tx = int(next_node[i][0])
        ty = int(next_node[i][1])
        #making sure values stay within bounds
        if tx < 0 or ty < 0 or tx > 600 or ty > 250:
            temp_node.remove(next_node[i])
        if map[tx,ty,0] != 0:
            temp_node.remove(next_node[i])
    return temp_node

#calculate cost to come for each node
def CalcCost(child, parent):
    xc = child[0]
    yc = child[1]
    xp = parent[0]
    yp = parent[1]
    lx = np.abs(xp-xc)
    ly =np.abs(yp-yc)
    cost = round(np.sqrt(lx**2 + ly**2),1)
    return cost

#calculate cost to go for each node
def CalcCostGo(node, goal):
    x_dist = goal[0] - node[0]
    y_dist = goal[1] - node[1]
    dist = np.sqrt(x_dist**2 + y_dist**2)
    return dist

#create empty map
map_empty = np.zeros([600, 250, 3], dtype=np.uint8)
map_copy = map_empty.copy()

#5mm buffer as called for
buffer = 5


#Creating all obstacles on map
#creating square shape dimensions and assigning pixel values to map
x_square_start = 100
x_square_end = 150
y_square_start = 0
y_square_end = 100
for i in range(x_square_start, x_square_end):
    for j in range(y_square_start, y_square_end):
        map_empty[i,j,:] = [239,76,76]

#plotting buffer zone
for i in range(x_square_start-buffer, x_square_end+buffer):
    for j in range(y_square_start-buffer, y_square_end+buffer):
            if map_empty[i,j,0] != 239:
                map_empty[i,j,:] = [239,76,76]

x_square_start = 100
x_square_end = 150
y_square_start = 150
y_square_end = 249
for i in range(x_square_start, x_square_end):
    for j in range(y_square_start, y_square_end):
        map_empty[i,j,:] = [239,76,76]

#plotting buffer zone
for i in range(x_square_start-buffer, x_square_end+buffer):
    for j in range(y_square_start-buffer, y_square_end):
            if map_empty[i,j,0] != 239:
                map_empty[i,j,:] = [239,76,76]

#hexagon
x_square_start = 230
x_square_end = 370
y_square_start = 45
y_square_end = 205
for i in range(x_square_start, x_square_end):
    for j in range(y_square_start, y_square_end):
        map_empty[i,j,:] = [239,76,76]


#creating diagonal wall dimensions and assigning pixel values to map
x_wall_start = 200
x_wall_end = 300
y_wall_start = 0
y_wall_end = 100
for i in range(x_wall_start, x_wall_end):
    for j in range(y_wall_start, y_wall_end):
        if -0.57*i < j and -0.57*i + 216 > j: 
            map_empty[i,j,:] = [0,0,0]

x_wall_start = 280
x_wall_end = 400
y_wall_start = 0
y_wall_end = 200
for i in range(x_wall_start, x_wall_end):
    for j in range(y_wall_start, y_wall_end):
        if 0.57*i - 125 > j and 0.57*i - 255 < j: 
            map_empty[i,j,:] = [0,0,0]

x_wall_start = 200
x_wall_end = 400
y_wall_start = 0
y_wall_end = 220
for i in range(x_wall_start, x_wall_end):
    for j in range(y_wall_start, y_wall_end):
        if 0.57*i + 80 > j and 0.57*i + 34 < j: 
            map_empty[i,j,:] = [0,0,0]

x_wall_start = 290
x_wall_end = 400
y_wall_start = 0
y_wall_end = 220
for i in range(x_wall_start, x_wall_end):
    for j in range(y_wall_start, y_wall_end):
        if -0.57*i + 375 < j and -0.57*i + 495 > j: 
            map_empty[i,j,:] = [0,0,0]


#triangle
x_wall_start = 460 - buffer
x_wall_end = 510 + buffer
y_wall_start = 125
y_wall_end = 240
for i in range(x_wall_start, x_wall_end):
    for j in range(y_wall_start, y_wall_end):
        if -7/4*i < j and -7/4*i + 1026 > j: 
            map_empty[i,j,:] = [239,76,76]

x_wall_start = 460 - buffer
x_wall_end = 510 + buffer
y_wall_start = 0
y_wall_end = 125
for i in range(x_wall_start, x_wall_end):
    for j in range(y_wall_start, y_wall_end):
        if 7/4*i - 776 < j and 7/4*i > j: 
            map_empty[i,j,:] = [239,76,76]

for i in range(0,600):
    for j in range(0,250):
        if i < 6:
            map_empty[i,j,:] = [239,76,76]
        if i > 594:
            map_empty[i,j,:] = [239,76,76]
        if j < 6:
            map_empty[i,j,:] = [239,76,76]
        if j > 244:
            map_empty[i,j,:] = [239,76,76]


#assigning start and end points
print('Enter step size:')
l = int(input())
print('Enter start node x:')
sx = int(input())
print('Enter start node y:')
sy = int(input())
print('Enter starting angle (0 degreees points East):')
sa = int(input())
print('Enter goal node x:')
gx = int(input())
print('Enter goal node y:')
gy = int(input())
print('Enter goal angle (0 degreees points East):')
ga = int(input())
print('Start Node:',sx, sy, sa)
print('Goal Node:',gx, gy, ga)
print('Step Size:',l)
#assign values and dimensions
Xs = [sx, sy, sa]
goal = [gx,gy]
goal_angle = ga
dim = (600,250)
#l = 5

#creating visit matrix
vdimx = int(600/0.5)
vdimy = int(250/0.5)
vdim = (vdimx,vdimy,12)
V = np.zeros(vdim)

##checking for goalpoint or start point in obstacle
if map_empty[gx,gy,0] != 0:
    print("goal lies in obstacle")
    sys.exit()
if map_empty[sx,sy,0] != 0:
    print("start lies in obstacle")
    sys.exit()

#creating cost to come and total cost matricies
cost_map = np.zeros(dim)
for i in range(0, 600):
    for j in range(0,250):
        if map_empty[i,j,0] == 0:
            cost_map[i,j] = 1E9
        else:
            cost_map[i,j] = -1

total_cost_map = np.zeros(dim)
for i in range(0, 600):
    for j in range(0,250):
        if map_empty[i,j,0] == 0:
            total_cost_map[i,j] = 1E9
        else:
            total_cost_map[i,j] = -1

#initialize variables
goal_state = 1
OpenList = PriorityQueue()
open_list = []
ClosedList = []
closed_list = []
cost_map[Xs[0],Xs[1]] = 0
goal_thresh = 3
d = 1
parents = {}
c = 1
#start timer
start_time = time.time()

#begin A* algorithm
OpenList.put((0,Xs))
while OpenList and goal_state != 0:

    #start exploring node
    Node_State_i = OpenList.get()[1]

    #make sure node is marked as visited
    V[[int(round(Node_State_i[0]))],[int(round(Node_State_i[1]))], [int(Node_State_i[2]%30)]] = 1

    #if node is within goal threshold, end loop
    if Node_State_i[0] > goal[0]-goal_thresh and Node_State_i[1] > goal[1]-goal_thresh and Node_State_i[0] < goal[0]+goal_thresh and Node_State_i[1] < goal[1]+goal_thresh:
        print('SUCCESS')
        goal[0] = int(round(Node_State_i[0]))
        goal[1] = int(round(Node_State_i[1]))
        cost2come = l + cost_map[int(round(Node_State_i[0])),int(round(Node_State_i[1]))]
        cost_map[int(round(Node_State_i[0])),int(round(Node_State_i[1]))] = cost2come
        break
    
    #get every possible move from moveset
    testing_node = moveset(map_empty, Node_State_i, l)

    #loop through each possible move
    for item in testing_node:
        #if node hasn't been visited, calculate the costs and mark as visited
        if V[[int(round(item[0]))],[int(round(item[1]))], [int(item[2]%30)]] == 0:
            if item is not OpenList:
                V[[int(round(item[0]))],[int(round(item[1]))], [int(item[2]%30)]] = 1
                #c = c+1
                map_empty[int(round(item[0])),int(round(item[1])),1] = 255 ######
                # if c % 10 == 0:
                #     cv.imwrite("images/Frame%d.jpg"%d, map_empty)
                #     d = d+1
                cost2come = l + cost_map[int(round(Node_State_i[0])),int(round(Node_State_i[1]))]
                cost_map[int(round(item[0])),int(round(item[1]))] = cost2come
                cost2go = CalcCostGo(item,goal)
                cost = cost2come + cost2go
                total_cost_map[int(round(item[0])),int(round(item[1]))] = cost
                parents.setdefault((int(round(Node_State_i[0])),int(round(Node_State_i[1]))), [])
                parents[int(round(Node_State_i[0])),int(round(Node_State_i[1]))].append([int(round(item[0])),int(round(item[1]))])
                OpenList.put((cost,item))
        #if node has been visited, check to see if new total cost is less than the current total cost
        else:
            cost2come = l + cost_map[int(round(Node_State_i[0])),int(round(Node_State_i[1]))]
            cost2go = CalcCostGo(item,goal)
            cost = cost2come + cost2go
            #if new total cost is greater, update with lower value
            if total_cost_map[int(round(item[0])),int(round(item[1]))] > cost:
                total_cost_map[int(round(item[0])),int(round(item[1]))] = cost
                parents.setdefault((int(round(Node_State_i[0])),int(round(Node_State_i[1]))), [])
                parents[int(round(Node_State_i[0])),int(round(Node_State_i[1]))].append([int(round(item[0])),int(round(item[1]))])
    testing_node = []

#end timer and print
print("A* search took %s seconds" % (time.time() - start_time))

#Backtracking algorithm
key_list=list(parents.keys())
val_list=list(parents.values())

print("Generating Path...")
path = []
path.append(goal)
node = goal
start = [Xs[0], Xs[1]]
while node != start:
    for list in val_list:
        if node in list:
            ind = val_list.index(list)
            path.append(key_list[ind])
            node = [key_list[ind][0], key_list[ind][1]]
path.reverse()



#mark all visited nodes as green
for i in range(0, V.shape[0]):
    for j in range(0,V.shape[1]):
        for k in range(0,V.shape[2]):
            if V[i][j][k] == 1:
                map_empty[i,j,1] = 255

image = map_empty

for i in range(0,len(path) - 1):
    x1 = path[i][0]
    y1 = path[i][1]
    x2 = path[i+1][0]
    y2 = path[i+1][1]
    image = cv.arrowedLine(image, (y1,x1), (y2,x2), color=(0,0,255), thickness=1) 

#creating and displaying final map
map_final = cv.rotate(image, cv.ROTATE_90_COUNTERCLOCKWISE)
map_final = cv.cvtColor(map_final, cv.COLOR_RGB2BGR)

cv.imshow("A* Search with best path",map_final)
cv.waitKey(0)
cv.destroyAllWindows()
