import pygame
import math

pygame.init()
width, height = 600, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Bug1 Algorithm Simulation")

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
ROBOT_COLOR = (0, 100, 255)
GOAL_COLOR = (0, 255, 0)
OBSTACLE_COLOR = (200, 0, 0)
PATH_COLOR = (100, 100, 100)

robot_pos = [50, 550]
robot_radius = 8
step_size = 2
goal = [300, 50]

obstacles = [
    pygame.Rect(150, 200, 100, 50),
    pygame.Rect(300, 100, 50, 150),
    pygame.Rect(400, 350, 120, 60),
    pygame.Rect(200, 400, 60, 100)
]

def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def detect_collision(pos):
    point = pygame.Rect(pos[0]-robot_radius, pos[1]-robot_radius, robot_radius*2, robot_radius*2)
    return any(point.colliderect(obs) for obs in obstacles)

def move_towards(target, pos, heading=None):
    if heading is None:
        angle = math.atan2(target[1] - pos[1], target[0] - pos[0])
    else:
        angle = heading
    new_x = pos[0] + step_size * math.cos(angle)
    new_y = pos[1] + step_size * math.sin(angle)
    return [new_x, new_y], angle

state = "go_to_goal"
path = []

p_impact = None
p_min = None
d_min = float('inf')
initial_heading = None

running = True
clock = pygame.time.Clock()

while running:
    clock.tick(60)
    screen.fill(WHITE)
    
    for obs in obstacles:
        pygame.draw.rect(screen, OBSTACLE_COLOR, obs)
    
    pygame.draw.circle(screen, GOAL_COLOR, goal, robot_radius)
    
    if len(path) > 1:
        pygame.draw.lines(screen, PATH_COLOR, False, path, 2)

    if state == "go_to_goal":
        robot_pos, heading = move_towards(goal, robot_pos)
        path.append(tuple(robot_pos))
        if distance(robot_pos, goal) < robot_radius:
            print("Reached goal!")
            running = False
        elif detect_collision(robot_pos):
            state = "wall_follow"
            p_impact = robot_pos.copy()
            p_min = robot_pos.copy()
            d_min = distance(p_impact, goal)
            initial_heading = heading

    elif state == "wall_follow":
        test_heading = initial_heading
        for i in range(36):
            test_heading = initial_heading + math.radians(-10 * i)
            new_pos = [robot_pos[0] + step_size * math.cos(test_heading),
                       robot_pos[1] + step_size * math.sin(test_heading)]
            if not detect_collision(new_pos):
                robot_pos = new_pos
                initial_heading = test_heading
                break
        path.append(tuple(robot_pos))

        d_cur = distance(robot_pos, goal)
        if d_cur < d_min:
            d_min = d_cur
            p_min = robot_pos.copy()

        if distance(robot_pos, p_impact) < robot_radius:
            state = "go_to_pmin"
    
    elif state == "go_to_pmin":
        robot_pos, heading = move_towards(p_min, robot_pos)
        path.append(tuple(robot_pos))
        if distance(robot_pos, p_min) < robot_radius:
            state = "go_to_goal"
    
    pygame.draw.circle(screen, ROBOT_COLOR, (int(robot_pos[0]), int(robot_pos[1])), robot_radius)
    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

pygame.quit()
