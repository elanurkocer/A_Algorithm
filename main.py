import pygame
import heapq
import random

# Pygame başlat
pygame.init()

# Renkler
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)

# Ekran boyutları
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Robot Movement")

# Robot özellikleri
robot_width = 50
robot_height = 50
robot_pos = [100, 100]
robot_speed = 1  # Robotun hızı

# Robot resmini yükle
robot_image = pygame.image.load("robot.webp")  # Resim dosyasını yükle
robot_image = pygame.transform.scale(robot_image, (robot_width, robot_height))  # Boyutunu ayarla

# Hedef ve engeller
goal = (700, 500)
# 30 rastgele engel
obstacles = [pygame.Rect(random.randint(100, 700), random.randint(100, 500), 50, 50) for _ in range(30)]

# A* Algoritması
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def is_colliding_with_obstacle(rect, obstacles):
    for obstacle in obstacles:
        if rect.colliderect(obstacle):
            return True
    return False

def astar(start, goal):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []

    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data[::-1]  # Yolun tersine döndürülmesi

        close_set.add(current)
        for i, j in neighbors:
            neighbor = (current[0] + i, current[1] + j)
            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            # Robotun boyutlarını dikkate alarak çarpışma kontrolü
            robot_rect = pygame.Rect(neighbor[0], neighbor[1], robot_width, robot_height)
            if 0 <= neighbor[0] < SCREEN_WIDTH and 0 <= neighbor[1] < SCREEN_HEIGHT:
                if is_colliding_with_obstacle(robot_rect, obstacles):
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float("inf")):
                    continue

                if tentative_g_score < gscore.get(neighbor, float("inf")) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return []  # Yol bulunamazsa boş liste döndür

# Oyun döngüsü
running = True
clock = pygame.time.Clock()  # FPS kontrolü
path = astar(tuple(robot_pos), goal)  # A* algoritması ile yol hesapla
current_step = 0

# Hedefe ulaşma bayrağı
reached_goal = False

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if not reached_goal:
        # Eğer yol bulunduysa, adım adım ilerle
        if current_step < len(path):
            # Robotun hedefe doğru ilerlemesi için birden fazla adım at
            for _ in range(robot_speed):  # Robot hızı kadar adım at
                if current_step < len(path):
                    robot_pos = list(path[current_step])  # Adım adım ilerle
                    current_step += 1
                else:
                    break

        # Hedefe ulaştığında yarışma biter
        if pygame.Rect(robot_pos[0], robot_pos[1], robot_width, robot_height).colliderect(pygame.Rect(goal[0], goal[1], 50, 50)):
            reached_goal = True  # Hedefe ulaşıldı
            print("Robot hedefe ulaştı!")

    # Ekranı temizle
    screen.fill(WHITE)

    # Engelleri ve hedefi çiz
    pygame.draw.rect(screen, GREEN, pygame.Rect(goal[0], goal[1], 50, 50))
    for ob in obstacles:
        pygame.draw.rect(screen, BLACK, ob)

    # Robotu çiz
    screen.blit(robot_image, (robot_pos[0], robot_pos[1]))  # Resmi ekrana yerleştir

    # Skor hesaplama
    distance_to_goal = heuristic(tuple(robot_pos), goal)  # Mesafeyi hesapla
    score = max(1, 10 - (distance_to_goal // 10))  # 1-10 arası skor

    # Skor gösterimi
    font = pygame.font.Font(None, 36)
    text_score = font.render(f"Skor: {score}", True, BLACK)
    screen.blit(text_score, (10, 10))

    # Ekranı güncelle
    pygame.display.flip()

    # FPS kontrolü (30 FPS)
    clock.tick(30)

    # Hedefe ulaşıldıysa bekle ve kapan
    if reached_goal:
        pygame.time.wait(3000)  # 2 saniye bekle
        running = False

pygame.quit()
