# @junhee.lee 241031

import pygame
import casadi as ca
import numpy as np
from colorama import init, Fore, Style
from collections import deque

# 파라미터 설정
g = 9.81  # [m/s^2]

lf = 1.2    # 전축에서 무게중심까지의 거리 [m]
lr = 1.6    # 후축에서 무게중심까지의 거리 [m]
L = lf + lr # 차량의 휠베이스 [m]

max_steering_angle = np.deg2rad(45)  # 최대 조향각 [rad]

# 스케일링 인자
scale = 10  # 1미터당 10픽셀

def kinematic_bicycle_model():
    # CasADi 심볼릭 변수 정의
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    yaw = ca.SX.sym('yaw')
    v = ca.SX.sym('v')

    state = ca.vertcat(x, y, yaw, v)

    # 제어 입력
    delta = ca.SX.sym('delta')
    a = ca.SX.sym('a')
    u = ca.vertcat(delta, a)

    # 동역학 방정식 정의 (카이네마틱 자전거 모델)
    dx = v * ca.cos(yaw)
    dy = v * ca.sin(yaw)
    dyaw = (v / L) * ca.tan(delta)
    dv = a

    xdot = ca.vertcat(dx, dy, dyaw, dv)

    # CasADi 통합기 설정
    dt = 0.02  # 시간 간격 [s]
    ode = {'x': state, 'p': u, 'ode': xdot}
    integrator = ca.integrator('integrator', 'rk', ode, {'tf': dt})

    return integrator, state

m = 2520    # 차량 질량 [kg]
Iz = 3600   # 차량 관성 모멘트 [kg·m²]

ro = 1.225          # 공기 밀도 [kg/m³]
Area = 2.8          # 전면 면적 [m²]
Cd = 0.35           # 항력 계수

Bf = 10.0  # 앞바퀴 스프링 계수
Cf = 1.3   # 앞바퀴 형태 계수
Df = 12552.0 # 앞바퀴 피크 값
Ef = 0.98  # 앞바퀴 곡률 계수

Br = 10.0   # 뒷바퀴 스프링 계수
Cr = 1.6     # 뒷바퀴 형태 계수
Dr = 20552.0 # 뒷바퀴 피크 값
Er = 0.98   # 뒷바퀴 곡률 계수


def dynamic_bicycle_model():
    # CasADi 심볼릭 변수 정의
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    yaw = ca.SX.sym('yaw')
    vx = ca.SX.sym('vx')
    vy = ca.SX.sym('vy')
    yawrate = ca.SX.sym('yawrate')

    state = ca.vertcat(x, y, yaw, vx, vy, yawrate)

    # 제어 입력
    delta = ca.SX.sym('delta')
    a = ca.SX.sym('a')
    u = ca.vertcat(delta, a)

    # Help variables 
    Fx_r = m * a 
    Faero = 0.5 * ro * Area * Cd * vx**2  # 항력 효과

    # Lateral Behavior
    alpha_f = ca.if_else(vx > 0.001, delta - ca.arctan((vy + lf * yawrate) / vx), 0.0)
    alpha_r = ca.if_else(vx > 0.001, ca.arctan((lr * yawrate - vy) / vx), 0.0)

    # Lateral Tire Forces (Pacejka 'magic formula')
    Fy_f = Df * ca.sin(Cf * ca.arctan(Bf * alpha_f - Ef * (Bf * alpha_f - ca.arctan(Bf * alpha_f))))
    Fy_r = Dr * ca.sin(Cr * ca.arctan(Br * alpha_r - Er * (Br * alpha_r - ca.arctan(Br * alpha_r))))

    # 동역학 방정식 정의 (다이나믹 자전거 모델)
    dx = vx * ca.cos(yaw) - vy * ca.sin(yaw)
    dy = vx * ca.sin(yaw) + vy * ca.cos(yaw)
    dyaw = yawrate
    dvx = (Fx_r - Faero - Fy_f * ca.sin(delta) + m * vy * yawrate) / m
    dvy = (Fy_r + Fy_f * ca.cos(delta) - m * vx * yawrate) / m
    dyawrate = (lf * Fy_f * ca.cos(delta) - lr * Fy_r) / Iz

    xdot = ca.vertcat(dx, dy, dyaw, dvx, dvy, dyawrate)

    # CasADi 통합기 설정
    dt = 0.02  # 시간 간격 [s]
    ode = {'x': state, 'p': u, 'ode': xdot}
    integrator = ca.integrator('integrator', 'rk', ode, {'tf': dt})

    return integrator, state

def dynamic_bicycle_model_rear():
    # CasADi 심볼릭 변수 정의
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    yaw = ca.SX.sym('yaw')
    vx = ca.SX.sym('vx')
    vy = ca.SX.sym('vy')
    yawrate = ca.SX.sym('yawrate')

    state = ca.vertcat(x, y, yaw, vx, vy, yawrate)

    # 제어 입력
    delta = ca.SX.sym('delta')
    a = ca.SX.sym('a')
    u = ca.vertcat(delta, a)

    # Help variables 
    Fx_r = m * a 
    Faero = 0.5 * ro * Area * Cd * vx**2  # 항력 효과

    # Lateral Behavior
    alpha_f = ca.if_else(vx > 0.001, delta - ca.arctan((vy + L * yawrate) / vx), 0.0)
    alpha_r = ca.if_else(vx > 0.001, ca.arctan(- vy / vx), 0.0)

    # Lateral Tire Forces (Pacejka 'magic formula')
    Fy_f = Df * ca.sin(Cf * ca.arctan(Bf * alpha_f - Ef * (Bf * alpha_f - ca.arctan(Bf * alpha_f))))
    Fy_r = Dr * ca.sin(Cr * ca.arctan(Br * alpha_r - Er * (Br * alpha_r - ca.arctan(Br * alpha_r))))

    # 동역학 방정식 정의 (다이나믹 자전거 모델)
    dx = vx * ca.cos(yaw) - vy * ca.sin(yaw)
    dy = vx * ca.sin(yaw) + vy * ca.cos(yaw)
    dyaw = yawrate
    dvx = (Fx_r - Faero - Fy_f * ca.sin(delta) + m * vy * yawrate) / m
    dvy = (Fy_r + Fy_f * ca.cos(delta) - m * vx * yawrate) / m
    dyawrate = (lf * Fy_f * ca.cos(delta)- lr * Fy_r) / Iz

    xdot = ca.vertcat(dx, dy, dyaw, dvx, dvy, dyawrate)

    # CasADi 통합기 설정
    dt = 0.02  # 시간 간격 [s]
    ode = {'x': state, 'p': u, 'ode': xdot}
    integrator = ca.integrator('integrator', 'rk', ode, {'tf': dt})

    return integrator, state



# CasADi 모델 생성
# integrator, state = kinematic_bicycle_model()
integrator, state = dynamic_bicycle_model()
# integrator, state = dynamic_bicycle_model_rear()

# 오도메트리 히스토리 저장 (이전 2초간의 데이터, 100포인트)
odometry_history = deque(maxlen=100)

# Pygame 초기화
pygame.init()
screen_width, screen_height = 1600, 1200
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Vehicle Simulator")
clock = pygame.time.Clock()

# 폰트 초기화
pygame.font.init()
font = pygame.font.SysFont(None, 24)

# 차량 크기 설정 (실제 차량 크기 기반)
car_length_m = 4.8  # 차량 길이 [m]
car_width_m = 2.3   # 차량 너비 [m]
tire_width_m = 0.5  # 타이어 너비 [m]
tire_radius_m = 0.8  # 타이어 반지름 [m]
car_length = car_length_m * scale  # 픽셀 단위 차량 길이
car_width = car_width_m * scale    # 픽셀 단위 차량 너비
tire_width = tire_width_m * scale
tire_radius = tire_radius_m * scale

# 차량 초기 상태
def reset_simulation():
    global state_k, delta_current, a_current, odometry_history
    state_k = np.zeros(state.size1())
    state_k[0] = screen_width / (2 * scale)
    state_k[1] = screen_height / (2 * scale)
    delta_current = 0.0  # 초기 조향각
    a_current = 0.0      # 초기 가속도
    odometry_history.clear()

reset_simulation()

# 초기 제어 입력
delta_current = 0.0  # 초기 조향각 [rad]
a_current = 0.0      # 초기 가속도 [m/s²]

# 제어 입력 제한 설정
max_acceleration = 5.0  # 최대 가속도 [m/s²]
steering_increment = np.deg2rad(2)   # 조향각 변화량 per frame [rad]
acceleration_increment = 0.5         # 가속도 변화량 per frame [m/s²]

# 뷰 모드 토글 변수 (True: 차량 중심, False: 전역 좌표계)
view_mode_vehicle_centered = True

# 키 입력 방법 정보
instruction_text = [
    "Controls:",
    "↑ : Accelerate",
    "↓ : Decelerate",
    "← : Turn Left",
    "→ : Turn Right",
    "B : Toggle View Mode (Vehicle-Centered / Global)",
    "Space : Reset Simulation"
]

# 키 입력 방법을 터미널에 노란색으로 출력
print(Fore.YELLOW + "\n".join(instruction_text) + Style.RESET_ALL + "\n")

# 메인 루프
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # 키보드 이벤트 처리
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                reset_simulation()
            elif event.key == pygame.K_b:
                # 뷰 모드 토글
                view_mode_vehicle_centered = not view_mode_vehicle_centered

    # 키보드 입력 처리
    keys = pygame.key.get_pressed()

    # 가속도 입력 업데이트
    if keys[pygame.K_UP]:
        a_current = min(a_current + acceleration_increment, max_acceleration)
    elif keys[pygame.K_DOWN]:
        a_current = max(a_current - acceleration_increment, -max_acceleration)
    else:
        # 가속도 점진적으로 감소 (마찰 효과)
        if a_current > 0:
            a_current = max(a_current - acceleration_increment, 0)
        elif a_current < 0:
            a_current = min(a_current + acceleration_increment, 0)

    # 조향각 입력 업데이트
    if keys[pygame.K_RIGHT]:
        delta_current = max(delta_current - steering_increment, -max_steering_angle)
    elif keys[pygame.K_LEFT]:
        delta_current = min(delta_current + steering_increment, max_steering_angle)
    else:
        # 조향각 점진적으로 복원
        if delta_current > 0:
            delta_current = max(delta_current - steering_increment, 0)
        elif delta_current < 0:
            delta_current = min(delta_current + steering_increment, 0)

    # 차량 동역학 업데이트
    res = integrator(x0=state_k, p=[delta_current, a_current])
    state_k = res['xf'].full().flatten()

    # 오도메트리 히스토리에 현재 위치 추가
    odometry_history.append((state_k[0], state_k[1]))

    # 화면 그리기
    screen.fill((255, 255, 255))  # 흰색 배경

    # 뷰 모드에 따른 차량 및 그리드 위치 계산
    if view_mode_vehicle_centered:
        # 차량 중심 뷰
        center_x = screen_width / 2
        center_y = screen_height / 2

        # 차량 위치는 화면 중앙으로 고정
        screen_x = center_x
        screen_y = center_y

        # 그리드 오프셋 계산
        grid_offset_x = -state_k[0] * scale + center_x
        grid_offset_y = state_k[1] * scale + center_y
    else:
        # 전역 좌표계 뷰
        screen_x = state_k[0] * scale
        screen_y = screen_height - state_k[1] * scale  # y좌표 반전

        grid_offset_x = 0
        grid_offset_y = 0

    # 그리드 그리기
    grid_color = (200, 200, 200)  # 회색 그리드 색상
    grid_size_m = 5  # 그리드 간격 [m]
    grid_size = grid_size_m * scale  # 그리드 간격 [픽셀]

    # 그리드 범위 계산
    start_x = int(-grid_size + grid_offset_x % grid_size)
    end_x = screen_width + grid_size
    start_y = int(-grid_size + grid_offset_y % grid_size)
    end_y = screen_height + grid_size

    for x in range(start_x, end_x, grid_size):
        pygame.draw.line(screen, grid_color, (x, 0), (x, screen_height))
    for y in range(start_y, end_y, grid_size):
        pygame.draw.line(screen, grid_color, (0, y), (screen_width, y))

    # 이전 2초간의 오도메트리 경로 그리기
    if len(odometry_history) > 1:
        points = []
        for pos in odometry_history:
            px, py = pos
            if view_mode_vehicle_centered:
                # 차량 중심 뷰에서는 그리드 오프셋을 적용
                px_screen = -px * scale + center_x
                py_screen = -py * scale + center_y
            else:
                # 전역 좌표계 뷰
                px_screen = px * scale
                py_screen = screen_height - py * scale
            points.append((px_screen, py_screen))
        if not view_mode_vehicle_centered: 
            # 경로 그리기 (파란색 선으로 표시)
            pygame.draw.lines(screen, (0, 0, 255), False, points, 2)  # 파란색 선, 두께 2

    # 차량 이미지를 그릴 표면 생성
    car_surface = pygame.Surface((car_length, car_width), pygame.SRCALPHA)

    # 차량 본체 그리기
    car_rect = car_surface.get_rect()
    pygame.draw.rect(car_surface, (255, 0, 0), car_rect)  # 빨간색 차량 본체

    # 타이어 위치 계산 (차량 좌표계에서)
    tire_positions = [
        # (x_offset, y_offset, angle)
        (lf * scale, car_width / 2 - tire_width / 2, delta_current),  # Front Left
        (lf * scale, -car_width / 2 + tire_width / 2, delta_current),  # Front Right
        (-lr * scale, car_width / 2 - tire_width / 2, 0.0),           # Rear Left
        (-lr * scale, -car_width / 2 + tire_width / 2, 0.0)           # Rear Right
    ]

    for x_offset, y_offset, tire_angle in tire_positions:
        # 타이어 위치
        tire_x = car_rect.centerx + x_offset
        tire_y = car_rect.centery + y_offset

        # 타이어 표면 생성
        tire_surface = pygame.Surface((tire_radius, tire_width), pygame.SRCALPHA)
        tire_rect = tire_surface.get_rect(center=(tire_x, tire_y))

        # 타이어 그리기
        pygame.draw.rect(tire_surface, (0, 0, 0), tire_surface.get_rect())

        # 타이어 회전
        rotated_tire = pygame.transform.rotate(tire_surface, np.rad2deg(tire_angle))

        # 회전된 타이어의 중심 위치 보정
        rotated_tire_rect = rotated_tire.get_rect(center=(tire_x, tire_y))

        # 차량 표면에 타이어 그리기
        car_surface.blit(rotated_tire, rotated_tire_rect)

    # 차량 회전
    rotated_car = pygame.transform.rotate(car_surface, np.rad2deg(state_k[2]))

    # 회전된 차량 그리기
    car_rect = rotated_car.get_rect(center=(screen_x, screen_y))
    screen.blit(rotated_car, car_rect)

    # 현재 속도 및 조향각 계산
    speed = np.sqrt(state_k[3]**2) # v = sqrt(vx²)
    speed_kph = speed * 3.6  # m/s를 km/h로 변환
    steering_angle_deg = np.degrees(delta_current)  # delta

    # 현재 뷰 모드 표시
    view_mode_text = "Vehicle-Centered View" if view_mode_vehicle_centered else "Global View"

    # 정보 표시
    info_text = [
        f"View Mode: {view_mode_text}",
        f"Position: X={state_k[0]:.2f} m, Y={state_k[1]:.2f} m, Yaw={np.degrees(state_k[2]) % 360:.2f}°",
        f"Steering Angle: {steering_angle_deg:.2f}°",
        f"Speed: {speed_kph:.2f} kph"
    ]

    for i, text in enumerate(info_text):
        rendered_text = font.render(text, True, (0, 0, 0))
        screen.blit(rendered_text, (10, 10 + i * 30))  # 간격을 30으로 늘려 가독성 향상

    pygame.display.flip()
    clock.tick(50)  # 50 FPS로 설정 (dt=0.02)

pygame.quit()
