import random
import numpy as np
from math import *
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import heapq
from copy import deepcopy


np.random.seed(0)



# 맵 크기 설정
MAP_WIDTH = 25  # 가로 25미터
MAP_HEIGHT = 15  # 세로 15미터
CELL_SIZE = 1.0  # 각 셀의 크기 (1미터)


START = (1.5,13.5)


# 맵 생성
grid_map = np.zeros((MAP_HEIGHT, MAP_WIDTH))

def visualize_grid_map(map_data, robot=None, particles=None):
    plt.figure(figsize=(12, 7))
    

    colors = ['white', 'black', 'lightblue', 'blue']
    custom_cmap = ListedColormap(colors)
    
    # 맵 표시
    plt.imshow(map_data, cmap=custom_cmap, vmin=0, vmax=3, origin='lower', extent=[0, MAP_WIDTH, 0, MAP_HEIGHT])

    plt.text(1.5,13.5,'A',ha='center',va='center', color='black', fontsize=16, fontweight='bold')
    plt.text(14.5,1.5,'P1',ha='center',va='center', color='black', fontsize=13, fontweight='bold')
    plt.text(18.5,1.5,'P2',ha='center',va='center', color='black', fontsize=13, fontweight='bold')



    if particles is not None:
        particle_x = [p.x for p in particles]
        particle_y = [p.y for p in particles]
        plt.scatter(particle_x, particle_y, color='green', alpha=0.3, s=10, label='Particles')

    # 로봇 위치 표시
    if robot is not None:
        plt.scatter(robot.x, robot.y, color='blue', s=100, label='Robot')
        # 로봇의 방향 표시 (작은 선으로)
        arrow_length = 0.5
        dx = arrow_length * np.cos(robot.orientation)
        dy = arrow_length * np.sin(robot.orientation)
        plt.arrow(robot.x, robot.y, dx, dy, head_width=0.2, head_length=0.2, fc='blue', ec='blue')




    # 그리드 라인 추가
    # 세로선
    for x in range(MAP_WIDTH + 1):
        plt.axvline(x, color='gray', linewidth=1)
    # 가로선
    for y in range(MAP_HEIGHT + 1):
        plt.axhline(y, color='gray', linewidth=1)
    
    # 축 눈금 설정
    plt.xticks(range(MAP_WIDTH+1))
    plt.yticks(range(MAP_HEIGHT+1))
    
    # 축 레이블
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    
    # 제목
    plt.title('Grid Map (1m x 1m cells)')
    
    #plt.grid(True)
    plt.show()



# 장애물 표시
grid_map[9:13, 3:5] = 2
grid_map[9:13, 8:10] = 2
grid_map[9:13, 14:16] = 2
grid_map[6:13, 18] = 2
grid_map[5:7, 3:7] = 2
grid_map[6, 10:19] = 2
grid_map[2:4, 12:14] = 2
grid_map[2:4, 16:18] = 2
grid_map[2:4, 20:22] = 2
grid_map[8:10, 21:25] = 2

# 랜드마크 표시
grid_map[4,5]=3
grid_map[9,16]=3
grid_map[11,5]=3
grid_map[2,15]=3


# 맵 시각화
visualize_grid_map(grid_map)

# 특정 셀의 실제 미터 단위 좌표 확인을 위한 함수
def get_cell_center(cell_x, cell_y):
    """셀의 중심점 좌표를 반환"""
    return cell_x + 0.5, cell_y + 0.5



landmark1 = get_cell_center(5,4)
landmark2 = get_cell_center(16,9)
landmark3 = get_cell_center(5,11)
landmark4 = get_cell_center(15,2)




START_CENTER = get_cell_center(START[0],START[1])


LANDMARKS = np.array([landmark1, landmark2, landmark3, landmark4])



# bycicle 모델

class Robot(object):

    def __init__(self, length=0.5):

        """

        Creates robot and initializes location/orientation to 0, 0, 0.

        """

        self.movement_pattern = [(1,0), (0,1), (-1,0), (0,-1)]  # 시계방향 순환
        self.pattern_index = 0


        self.x = random.random() * MAP_WIDTH

        self.y = random.random() * MAP_HEIGHT

        # self.orientation = random.random() * 2.0 * pi


        self.orientation = 0.0

        self.length = length

        self.steering_noise = 0.07

        self.distance_noise = 0.1

        self.steering_drift = 0.0

        self.measurement_noise = 0.4

        self.measurement_range = 5.0

    def set(self, x, y, orientation=0.0):

        """

        Sets a robot coordinate.

        """

        self.x = x

        self.y = y

        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise, measurement_noise):

        """

        Sets the noise parameters.

        """

        # makes it possible to change the noise parameters

        # this is often useful in particle filters

        self.steering_noise = steering_noise

        self.distance_noise = distance_noise

        self.measurement_noise = measurement_noise

    def set_steering_drift(self, drift):

        """

        Sets the systematical steering drift parameter

        """

        self.steering_drift = drift


    def is_valid_position(self, x, y):
        """주어진 위치가 유효한지 확인"""
        # 맵 경계 확인
        if x < 0 or x >= MAP_WIDTH or y < 0 or y >= MAP_HEIGHT:
            return False
        
        # 격자 좌표로 변환
        grid_x = int(x)
        grid_y = int(y)
        
        # 장애물이나 랜드마크 확인
        if grid_map[grid_y, grid_x] in [2, 3]:  # 2는 장애물, 3은 랜드마크
            return False
        
        return True


    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):



        if steering > max_steering_angle:

            steering = max_steering_angle

        if steering < -max_steering_angle:

            steering = -max_steering_angle


        '''
        if distance < 0.0:

            distance = 0.0
        '''


        # apply noise

        steering2 = random.gauss(steering, self.steering_noise)

        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift

        steering2 += self.steering_drift

        # Execute motion

        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # 새로운 위치 계산
            new_x = self.x + distance2 * np.cos(self.orientation)
            new_y = self.y + distance2 * np.sin(self.orientation)
            new_orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            new_orientation = (self.orientation + turn) % (2.0 * np.pi)
            new_x = cx + (np.sin(new_orientation) * radius)
            new_y = cy - (np.cos(new_orientation) * radius)

        # 충돌 회피 로직 추가
        if not self.is_valid_position(new_x, new_y):
            # 1. 후진 시도
            backup_distance = -0.5 * distance
            backup_x = self.x + backup_distance * np.cos(self.orientation)
            backup_y = self.y + backup_distance * np.sin(self.orientation)
            
            if self.is_valid_position(backup_x, backup_y):
                res = Robot()
                res.set(backup_x, backup_y, self.orientation)
                res.set_noise(self.steering_noise, self.distance_noise, self.measurement_noise)
                return res
                
            # 2. 제자리 회전 시도
            turn_angles = [pi/4, -pi/4, pi/2, -pi/2]
            for angle in turn_angles:
                new_orientation = (self.orientation + angle) % (2.0 * np.pi)
                test_x = self.x + 0.1 * np.cos(new_orientation)
                test_y = self.y + 0.1 * np.sin(new_orientation)
                
                if self.is_valid_position(test_x, test_y):
                    res = Robot()
                    res.set(self.x, self.y, new_orientation)
                    res.set_noise(self.steering_noise, self.distance_noise, self.measurement_noise)
                    return res
            
            # 3. 모든 시도가 실패하면 현재 위치 유지
            return self

        res = Robot()
        res.set(new_x, new_y, new_orientation)
        res.set_noise(self.steering_noise, self.distance_noise, self.measurement_noise)
        return res
    


    def move_discrete(self):
        """정해진 패턴으로 이동 (시계방향 순환)"""
        dx, dy = self.movement_pattern[self.pattern_index]
        
        # 다음 위치 계산
        target_x = self.x + dx
        target_y = self.y + dy
        
        # 이동이 유효하면 실행
        if self.is_valid_position(target_x, target_y):
            target_orientation = atan2(dy, dx)
            steering = (target_orientation - self.orientation) % (2.0 * np.pi)
            if steering > np.pi:
                steering -= 2.0 * np.pi
                
            # 다음 패턴으로 이동
            self.pattern_index = (self.pattern_index + 1) % len(self.movement_pattern)
            return self.move(steering, 1.0)
            
        # 이동이 불가능하면 다음 패턴으로
        self.pattern_index = (self.pattern_index + 1) % len(self.movement_pattern)
        return self


    def sense(self):

        Z = []

        for i, landmark in enumerate(LANDMARKS):

            dist = sqrt((self.x - landmark[0]) ** 2 + (self.y - landmark[1]) ** 2)
            if dist <= self.measurement_range:
                dist += random.gauss(0.0, self.measurement_noise)
            else:
                dist = float('inf')
            Z.append(dist)

        # 2. 장애물 센싱 (4방향)
        directions = [(1,0), (0,1), (-1,0), (0,-1)]  # 동,북,서,남
        for dx, dy in directions:
            dist = self.calculate_obstacle_distance(dx, dy)
            Z.append(dist)

        return Z
    

    def Gaussian(self, mu, sigma, x):

        return exp(-((mu-x)**2)/(sigma**2)/2)/sqrt(2.0*pi*(sigma**2))


    def measure_prob(self, measure):
        if len(measure) != 8:  # 안전 검사 추가
            print(f"Error: Expected 8 measurements, got {len(measure)}")
            return 0.0
        
        prob = 1.0
        
        # 1. 랜드마크에 대한 확률 계산 (처음 4개 값)
        for i in range(4):  # len(LANDMARKS) 대신 명시적으로 4 사용
            dist = sqrt((self.x-LANDMARKS[i][0])**2 + (self.y-LANDMARKS[i][1])**2)
            if dist <= self.measurement_range:
                if measure[i] != float('inf'):
                    prob *= self.Gaussian(dist, self.measurement_noise, measure[i])
                else:
                    prob *= 0.000001
            else:
                if measure[i] == float('inf'):
                    prob *= 1.0
                else:
                    prob *= 0.000001
        
        # 2. 장애물에 대한 확률 계산 (나머지 4개 값)
        directions = [(1,0), (0,1), (-1,0), (0,-1)]
        for i, (dx, dy) in enumerate(directions):
            measured_dist = measure[4 + i]  # 수정된 인덱싱
            expected_dist = self.calculate_obstacle_distance(dx, dy)
            
            if measured_dist != float('inf') and expected_dist != float('inf'):
                prob *= self.Gaussian(expected_dist, self.measurement_noise, measured_dist)
            elif measured_dist == expected_dist:  # 둘 다 inf인 경우
                prob *= 1.0
            else:
                prob *= 0.000001
                
        return prob
    


    def calculate_obstacle_distance(self, dx, dy):
        dist = 0
        curr_x, curr_y = self.x, self.y
        while dist <= self.measurement_range:
            curr_x += dx
            curr_y += dy
            dist += 1
            
            if curr_x < 0 or curr_x >= MAP_WIDTH or curr_y < 0 or curr_y >= MAP_HEIGHT:
                return float('inf')
            
            grid_x, grid_y = int(curr_x), int(curr_y)
            if grid_map[grid_y, grid_x] == 2:
                return dist
            
        return float('inf')
    

    def __repr__(self):

        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)





def eval(r,p):

  sum = 0.0

  for i in range(len(p)):


    dx = (p[i].x - r.x )

    dy = (p[i].y - r.y )

    err = sqrt(dx * dx + dy * dy)

    sum += err

  return sum/float(len(p))


def plotParticle(myrobot,p):

    plt.figure()

    plt.scatter(*zip(*LANDMARKS), marker='x', color='red', s=100, label='Landmarks')

    plt.scatter(myrobot.x, myrobot.y, marker='o', color='blue', s=100, label='True Robot Position')

    plt.scatter([particle.x for particle in p], [particle.y for particle in p], color='green', alpha=0.3, label='Particles')

    plt.xlim(-5, MAP_WIDTH+5)

    plt.ylim(-5, MAP_HEIGHT+5)

    plt.xlabel('X')

    plt.ylabel('Y')

    plt.title('Step: ' + str(t))

    plt.legend()

    plt.show()







robot = Robot()

robot.set(START[0],START[1],1.5*np.pi)

print("start 지점: ", robot)







Z = robot.sense()

N = 1000

T = 20

# 1000 Partilces are initialized

p = []



for i in range(N):

    r = Robot()

    while True:
        x = random.random() * MAP_WIDTH
        y = random.random() * MAP_HEIGHT
        if r.is_valid_position(x, y):  # 장애물과 랜드마크가 없는 셀에만 생성
            break
    r.set(x, y, random.random() * 2.0 * pi)

    p.append(r)

#print(len(p))


for t in range(T):

  robot = robot.move_discrete()

  Z = robot.sense()

  p = [e.move_discrete() for e in p]
  w = [e.measure_prob(Z) for e in p]
    
  # plotParticle(robot,p)

  visualize_grid_map(grid_map, robot, p)

  print('Localization error: ', eval(robot, p))

  ## Resampling

  p3 = []

  index = int(random.random() * N)

  beta = 0.0

  mw = max(w)

  for i in range(N):

    beta += random.random() * 2.0 * mw

    while beta > w[index]:

      beta -= w[index]

      index = (index + 1) % N

    p3.append(p[index])

  p = p3





def estimate_position(p, w):
    # 가중치의 합으로 정규화
    w_sum = sum(w)
    normalized_w = [weight/w_sum for weight in w]
    
    # 가중 평균으로 위치 추정
    estimated_x = sum(particle.x * w for particle, w in zip(p, normalized_w))
    estimated_y = sum(particle.y * w for particle, w in zip(p, normalized_w))
    
    return estimated_x, estimated_y

# 메인 루프 안에서 사용 (for t in range(T): 안에)
estimated_x, estimated_y = estimate_position(p, w)
print(f'로봇의 추정 위치: x={estimated_x:.2f}, y={estimated_y:.2f}')
print(f'실제 위치: x={robot.x:.2f}, y={robot.y:.2f}')








# ... 기존 코드 유지 ...

class Node:
    def __init__(self, position, g_cost=0, h_cost=0):
        self.position = position  # (x, y) 튜플
        self.g_cost = g_cost  # 시작점으로부터의 비용
        self.h_cost = h_cost  # 목표점까지의 추정 비용
        self.f_cost = g_cost + h_cost  # 총 비용
        self.parent = None

    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        return self.position == other.position and self.f_cost == other.f_cost
    


def manhattan_distance(start, goal):
    return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

def get_neighbors(position):
    x, y = position
    # 현재 위치의 셀 좌표 계산
    cell_x, cell_y = int(x - 0.5), int(y - 0.5)
    
    # 이웃 셀들의 중심 좌표 생성
    neighbors = [
        get_cell_center(cell_x + 1, cell_y),  # 오른쪽
        get_cell_center(cell_x - 1, cell_y),  # 왼쪽
        get_cell_center(cell_x, cell_y + 1),  # 위
        get_cell_center(cell_x, cell_y - 1)   # 아래
    ]
    
    valid_neighbors = []
    for nx, ny in neighbors:
        if is_valid_position(int(nx - 0.5), int(ny - 0.5)):  # 셀 좌표로 변환하여 확인
            valid_neighbors.append((nx, ny))
    
    return valid_neighbors


def is_valid_position(x, y):
    if x < 0 or x >= MAP_WIDTH or y < 0 or y >= MAP_HEIGHT:
        return False
    return grid_map[int(y), int(x)] not in [2, 3]  # 장애물이나 랜드마크가 아닌 경우만 통과

def find_path(start, goals):
    """A* 알고리즘을 사용하여 시작점에서 가장 가까운 목표점까지의 경로를 찾습니다."""

    start_cell_x, start_cell_y = int(start[0]), int(start[1])
    start = get_cell_center(start_cell_x, start_cell_y)
    
    print(f"경로 탐색 시작 - 시작 위치: {start}")
    
    # 각 목표점에 대해 경로 탐색
    best_path = None
    min_cost = float('inf')
    
    for goal in goals:
        goal_cell_x, goal_cell_y = int(goal[0]), int(goal[1])
        goal = get_cell_center(goal_cell_x, goal_cell_y)
        print(f"\n목표 지점 {goal}에 대한 경로 탐색 시작")

        open_list = []
        closed_set = set()
        
        start_node = Node(start, 0, manhattan_distance(start, goal))
        heapq.heappush(open_list, (start_node.f_cost, start_node))
        
        iterations = 0
        max_iterations = 1000  # 무한 루프 방지
        
        while open_list and iterations < max_iterations:
            iterations += 1
            current_f, current_node = heapq.heappop(open_list)
            
            if current_node.position == goal:
                path = []
                while current_node:
                    path.append(current_node.position)
                    current_node = current_node.parent
                path.reverse()
                print(f"목표 {goal}까지의 경로를 찾았습니다! 경로 길이: {len(path)}")
                
                if len(path) < min_cost:
                    min_cost = len(path)
                    best_path = path
                break
            
            if iterations % 100 == 0:  # 진행 상황 출력
                print(f"탐색 진행 중... {iterations}회 반복")
            
            closed_set.add(current_node.position)
            
            for neighbor_pos in get_neighbors(current_node.position):
                if neighbor_pos in closed_set:
                    continue
                
                g_cost = current_node.g_cost + 1
                h_cost = manhattan_distance(neighbor_pos, goal)
                
                neighbor_node = Node(neighbor_pos, g_cost, h_cost)
                neighbor_node.parent = current_node
                
                heapq.heappush(open_list, (neighbor_node.f_cost, neighbor_node))
        
        if iterations >= max_iterations:
            print(f"목표 {goal}에 대한 경로 탐색이 최대 반복 횟수를 초과했습니다.")
        elif not open_list:
            print(f"목표 {goal}에 대한 경로를 찾을 수 없습니다.")
    
    if best_path is None:
        print("어떤 목표 지점에도 도달할 수 있는 경로를 찾지 못했습니다.")
    
    return best_path





def smooth(path, weight_data = 0.5, weight_smooth = 0.55, tolerance = 0.0001):

    # Make a deep copy of path into newpath

    newpath = deepcopy(path)

    error1=10000

    error2=0
    while (abs(error1-error2) >= tolerance):

        error1=error2

        error2=0

        for i in range(1,len(path)-1):

            for k in range(2):

                d1=weight_data*(path[i][k]-newpath[i][k])

                d2=weight_smooth*(newpath[i+1][k]+newpath[i-1][k]-2*newpath[i][k])

                newpath[i][k]=newpath[i][k]+d1+d2

                error2 += d1**2+d2**2

    return newpath





def run_PID(robot, smoothed_path):
    # PID 파라미터 - 조향 응답성 증가
    steering_pid = {
        'P': 0.27,     # 0.15 -> 0.25로 증가 (더 빠른 반응)
        'I': 0.005,   # 적분 항 약간 증가
        'D': 1.7,      # 0.8 -> 0.7로 감소 (과도한 감쇠 방지)
        'error': 0.0,
        'error_sum': 0.0,
        'error_diff': 0.0,
        'prev_error': 0.0
    }
    
    def find_closest_forward_point(robot_pos, path, current_idx):
        min_dist = float('inf')
        best_idx = current_idx
        
        # 현재 인덱스부터 앞쪽의 경로 포인트들만 확인
        for i in range(current_idx, len(path)):
            dist = sqrt((robot_pos[0] - path[i][0])**2 + 
                       (robot_pos[1] - path[i][1])**2)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
        
        return best_idx
    
    robot_positions = []
    target_idx = 0
    stuck_counter = 0
    prev_position = (robot.x, robot.y)
    
    while target_idx < len(smoothed_path):
        robot_positions.append((robot.x, robot.y, robot.orientation))
        
        # 현재 로봇 위치에서 가장 가까운 앞쪽 경로 포인트 찾기
        target_idx = find_closest_forward_point(
            (robot.x, robot.y), 
            smoothed_path, 
            target_idx
        )
        
        # 현재 목표점 설정
        current_target = smoothed_path[target_idx]
        
        # 각도 오차 계산
        target_angle = atan2(current_target[1] - robot.y, 
                           current_target[0] - robot.x)
        angle_error = (target_angle - robot.orientation)
        
        # 각도 정규화 (-pi ~ pi)
        while angle_error > pi: angle_error -= 2*pi
        while angle_error < -pi: angle_error += 2*pi


        # 큰 회전각 제한
        if angle_error > pi/2:
            angle_error = pi/2
        elif angle_error < -pi/2:
            angle_error = -pi/2

        angle_diff = abs(angle_error)


        # PID 제어
        steering_pid['error'] = angle_error
        steering_pid['error_sum'] += angle_error
        steering_pid['error_diff'] = angle_error - steering_pid['prev_error']
        steering_pid['prev_error'] = angle_error
        

        # 적분 항 제한 (Anti-windup) 추가
        steering_pid['error_sum'] = max(min(steering_pid['error_sum'], 5.0), -5.0)


        # 조향각 계산
        steering = (steering_pid['P'] * steering_pid['error'] +
                   steering_pid['I'] * steering_pid['error_sum'] +
                   steering_pid['D'] * steering_pid['error_diff'])
        
        # 조향각 제한
        steering = max(min(steering, pi/6), -pi/6)  # pi/8 -> pi/6으로 증가
        


        # 속도 제어 로직 수정 - 커브 구간에서 더 빠르게
        base_speed = 0.25  # 0.15 -> 0.25로 증가

        # 회전각에 따른 속도 조정을 덜 보수적으로
        if angle_diff > pi/4:
            speed = base_speed * 0.4  # 0.2 -> 0.4로 증가
        elif angle_diff > pi/6:
            speed = base_speed * 0.6  # 0.4 -> 0.6으로 증가
        elif angle_diff > pi/8:
            speed = base_speed * 0.8  # 0.6 -> 0.8로 증가
        else:
            speed = base_speed

        # 조향각에 따른 속도 감소를 덜 급격하게
        speed = base_speed * (1.0 - 0.6 * abs(steering)/(pi/6))  # 0.8 -> 0.6으로 감소
        speed = max(0.1, speed)  # 최소 속도 약간 증가


        # 로봇 이동
        robot = robot.move(steering, speed)
        
        # 로봇이 움직이지 않는지 확인
        current_position = (robot.x, robot.y)
        if abs(current_position[0] - prev_position[0]) < 0.01 and \
           abs(current_position[1] - prev_position[1]) < 0.01:
            stuck_counter += 1
        else:
            stuck_counter = 0
        
        # 로봇이 일정 시간 동안 움직이지 않으면
        if stuck_counter > 10:
            # 현재 목표점을 건너뛰고 다음 목표점으로 이동
            target_idx += 1
            stuck_counter = 0
            steering_pid['error_sum'] = 0
            continue
            
        prev_position = current_position
        
        # 목표 지점 도달 확인
        distance_to_target = sqrt((robot.x - current_target[0])**2 + 
                                (robot.y - current_target[1])**2)
        
        # 목표 지점에 가까워질수록 속도 감소
        if distance_to_target < 1.0:
            speed *= (distance_to_target / 1.0)
            
        speed = max(0.1, speed)  # 최소 속도 보장

 
        
        if distance_to_target < 0.5:
            target_idx += 1
            steering_pid['error_sum'] = 0

        
        # 시각화
        plt.clf()  # 이전 그림 지우기
        
        # grid_map 표시
        colors = ['white', 'black', 'lightblue', 'blue']
        custom_cmap = ListedColormap(colors)
        plt.imshow(grid_map, cmap=custom_cmap, vmin=0, vmax=3, origin='lower', extent=[0, MAP_WIDTH, 0, MAP_HEIGHT])
        
        # 텍스트 표시
        plt.text(1.5,13.5,'A',ha='center',va='center', color='black', fontsize=16, fontweight='bold')
        plt.text(14.5,1.5,'P1',ha='center',va='center', color='black', fontsize=13, fontweight='bold')
        plt.text(18.5,1.5,'P2',ha='center',va='center', color='black', fontsize=13, fontweight='bold')
        
        # 계획된 경로 표시 (빨간색)
        path_x = [p[0] for p in smoothed_path]
        path_y = [p[1] for p in smoothed_path]
        plt.plot(path_x, path_y, 'r-', linewidth=2, label='Planned Path')
        
        # 로봇의 실제 이동 경로 표시 (초록색)
        robot_x = [p[0] for p in robot_positions]
        robot_y = [p[1] for p in robot_positions]
        plt.plot(robot_x, robot_y, 'g-', linewidth=2, label='Robot Trajectory')
        
        # 현재 로봇 위치 표시
        plt.scatter(robot.x, robot.y, color='blue', s=100, label='Robot')
        arrow_length = 0.5
        dx = arrow_length * np.cos(robot.orientation)
        dy = arrow_length * np.sin(robot.orientation)
        plt.arrow(robot.x, robot.y, dx, dy, head_width=0.2, head_length=0.2, fc='blue', ec='blue')
        
        # 그리드 라인 추가
        for x in range(MAP_WIDTH + 1):
            plt.axvline(x, color='gray', linewidth=1)
        for y in range(MAP_HEIGHT + 1):
            plt.axhline(y, color='gray', linewidth=1)
        
        plt.grid(True)
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.title('Robot Path Following with PID Control')
        plt.legend()
        
        # 축 범위 설정
        plt.xlim(0, MAP_WIDTH)
        plt.ylim(0, MAP_HEIGHT)
        
        plt.pause(0.1)
    
    plt.close()
    return robot_positions






# 주차 목표점 설정 (P1, P2)
parking_spots = [(14.5, 1.5), (18.5, 1.5)]

# 현재 로봇의 추정 위치에서 주차 지점까지의 경로 찾기
current_pos = (estimated_x, estimated_y)


print("현재 위치:", current_pos)
print("목표 위치들:", parking_spots)

# 시작 위치가 유효한지 확인
if not is_valid_position(round(current_pos[0]), round(current_pos[1])):
    print("시작 위치가 유효하지 않습니다!")



path = find_path(current_pos, parking_spots)

if path:
    print("경로를 찾았습니다!")
    print("경로:", path)
    
    # 맵 복사 및 시각화
    plt.figure(figsize=(12, 7))
    
    colors = ['white', 'black', 'lightblue', 'blue']
    custom_cmap = ListedColormap(colors)
    
    # 기본 맵 표시
    plt.imshow(grid_map, cmap=custom_cmap, vmin=0, vmax=3, origin='lower', extent=[0, MAP_WIDTH, 0, MAP_HEIGHT])
    

    plt.plot([robot.x, path[1][0]], [robot.y, path[1][1]], 'r-', linewidth=2)

    # 경로를 선으로 표시 (셀의 중심점들을 연결)
    path_x = [pos[0] for pos in path[1:]]
    path_y = [pos[1] for pos in path[1:]]
    plt.plot(path_x, path_y, 'b-', linewidth=2, label='Path')
    plt.scatter(path_x, path_y, color='blue', s=30)



    path_for_smoothing = [[pos[0], pos[1]] for pos in path[1:]]  # 두 번째 지점부터 사용
    
    # smooth 함수 적용
    smoothed_path = smooth(path_for_smoothing)

    plt.plot([robot.x, smoothed_path[0][0]], [robot.y, smoothed_path[0][1]], 'r-', linewidth=2)
    
    # 부드러운 경로 그리기
    smooth_x = [pos[0] for pos in smoothed_path]
    smooth_y = [pos[1] for pos in smoothed_path]
    plt.plot(smooth_x, smooth_y, 'r-', linewidth=2, label='Smoothed Path')
    plt.scatter(smooth_x, smooth_y, color='red', s=30)


    # 시작점과 목표점 표시
    plt.text(1.5, 13.5, 'A', ha='center', va='center', color='black', fontsize=16, fontweight='bold')
    plt.text(14.5, 1.5, 'P1', ha='center', va='center', color='black', fontsize=13, fontweight='bold')
    plt.text(18.5, 1.5, 'P2', ha='center', va='center', color='black', fontsize=13, fontweight='bold')
    
    # 로봇의 현재 위치 표시
    plt.scatter(robot.x, robot.y, color='blue', s=100, label='Robot')
    
    # 그리드 라인 추가
    for x in range(MAP_WIDTH + 1):
        plt.axvline(x, color='gray', linewidth=1)
    for y in range(MAP_HEIGHT + 1):
        plt.axhline(y, color='gray', linewidth=1)
    
    plt.xticks(range(MAP_WIDTH+1))
    plt.yticks(range(MAP_HEIGHT+1))
    
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Path to Parking Spot')
    plt.legend()
    plt.show()
    
    '''
    print(f"경로 길이: {len(path)} 스텝")
    print("상세 경로:")
    for i, (x, y) in enumerate(path):
        print(f"스텝 {i}: ({x}, {y})")
    
    '''
   
   



    robot_positions = run_PID(robot, smoothed_path)
    
    # 최종 결과 시각화
    plt.figure(figsize=(12, 7))
    colors = ['white', 'black', 'lightblue', 'blue']
    custom_cmap = ListedColormap(colors)
    
    plt.imshow(grid_map, cmap=custom_cmap, vmin=0, vmax=3, origin='lower', extent=[0, MAP_WIDTH, 0, MAP_HEIGHT])
    
    # 계획된 경로 표시
    plt.plot([p[0] for p in smoothed_path], [p[1] for p in smoothed_path], 'r-', linewidth=2, label='Planned Path')
    
    # 실제 로봇 궤적 표시
    plt.plot([p[0] for p in robot_positions], [p[1] for p in robot_positions], 'g-', linewidth=2, label='Robot Trajectory')
    
    plt.grid(True)
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Final Robot Path Following Result')
    plt.legend()
    plt.show()






else:
    print("경로를 찾을 수 없습니다.")