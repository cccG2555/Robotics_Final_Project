import random
import numpy as np
from math import *
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import matplotlib.animation as animation


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




# 맵 시각화
visualize_grid_map(grid_map)

# 특정 셀의 실제 미터 단위 좌표 확인을 위한 함수
def get_cell_center(cell_x, cell_y):
    """셀의 중심점 좌표를 반환"""
    return cell_x + 0.5, cell_y + 0.5





START_CENTER = get_cell_center(START[0],START[1])



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

        self.measurement_noise = 0.2

        self.measurement_range = 8.0


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
        
        # 장애물이 확인
        if grid_map[grid_y, grid_x] in [2]:  # 2는 장애물
            return False
        
        return True


    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):



        if steering > max_steering_angle:

            steering = max_steering_angle

        if steering < -max_steering_angle:

            steering = -max_steering_angle

        if distance < 0.0:

            distance = 0.0

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



        if not self.is_valid_position(new_x, new_y):
            return self

        res = Robot()
        res.set(new_x, new_y, new_orientation)
        res.set_noise(self.steering_noise, self.distance_noise, self.measurement_noise)
        return res
    


    def move_discrete(self):
        # 정해진 패턴으로 이동 (시계방향 순환)
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
        angles = np.arange(0, 360, 10)  # 10도 단위로 0도부터 359도까지
        
        for angle in angles:
            # 각도를 라디안으로 변환
            rad = np.radians(angle)
            dx = np.cos(rad)
            dy = np.sin(rad)
            
            # 각 방향에 대해 장애물까지의 거리 측정
            dist = self.calculate_distance_in_direction(dx, dy)
            Z.append(dist)
            
        return Z  # 36개의 측정값 반환 (360도/10도 = 36)

    def calculate_distance_in_direction(self, dx, dy):
        measurement_range = 8.0  # 8미터 측정 범위
        step_size = 0.1  # 정밀도를 위해 0.1m 단위로 검사
        
        dist = 0.0
        curr_x, curr_y = self.x, self.y
        
        while dist <= measurement_range:
            curr_x += dx * step_size
            curr_y += dy * step_size
            dist += step_size
            
            # 맵 경계 체크
            if curr_x < 0 or curr_x >= MAP_WIDTH or curr_y < 0 or curr_y >= MAP_HEIGHT:
                return measurement_range
            
            # 격자 좌표로 변환
            grid_x, grid_y = int(curr_x), int(curr_y)
            
            # 장애물 감지
            if grid_map[grid_y, grid_x] == 2:
                return dist
            
        return measurement_range

    def Gaussian(self, mu, sigma, x):

        return exp(-((mu-x)**2)/(sigma**2)/2)/sqrt(2.0*pi*(sigma**2))


    def measure_prob(self, measurement):
        if len(measurement) != 36:  # 36개의 측정값 확인
            print(f"Error: Expected 36 measurements, got {len(measurement)}")
            return 0.0
        
        prob = 1.0
        angles = np.arange(0, 360, 10)
        
        for i, angle in enumerate(angles):
            rad = np.radians(angle)
            dx = np.cos(rad)
            dy = np.sin(rad)
            
            expected_dist = self.calculate_distance_in_direction(dx, dy)
            measured_dist = measurement[i]
            
            # 측정값과 예상값의 차이에 대한 확률 계산
            prob *= self.Gaussian(expected_dist, self.measurement_noise, measured_dist)
        
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
        if r.is_valid_position(x, y):  # 장애물이 없는 셀에만 생성
            break
    r.set(x, y, random.random() * 2.0 * pi)

    p.append(r)




for t in range(T):
    # 로봇 이동
    robot = robot.move_discrete()
    
    # 파티클들도 동일한 move_discrete() 메소드로 이동
    p = [e.move_discrete() for e in p]
    
    Z = robot.sense()
    
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