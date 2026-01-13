from controller import Robot
import math
from collections import deque, defaultdict

# ============================================================
# 1) ДЕТЕКТОР ЦИЛИНДРОВ (твоя версия, без изменений логики)
# ============================================================
class CylinderScanner:
    def __init__(self, robot, camera_name='camera'):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())

        # --- КАМЕРА ---
        self.camera = robot.getDevice(camera_name)
        self.camera.enable(self.timestep)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()

        # --- ДАТЧИКИ ПОЛА ---
        self.gs_l = robot.getDevice('ground left infrared sensor')
        self.gs_r = robot.getDevice('ground right infrared sensor')
        self.gs_l.enable(self.timestep)
        self.gs_r.enable(self.timestep)

        self.count = 0
        self.is_tracking_object = False 
        self.frames_without_object = 0 
        self.ignore_start = 50 
        self.finished = False
        self.active = False 

    def update_and_check(self):
        if not self.active or self.finished:
            return False

        if self.ignore_start > 0:
            self.ignore_start -= 1
            return False

        yellow_detected = self._is_yellow_in_view()

        if yellow_detected:
            self.frames_without_object = 0 
            if not self.is_tracking_object:
                self.count += 1
                self.is_tracking_object = True
                print(f"Объект обнаружен! Текущее количество: {self.count}")
                
                # Лимит счетчика равен 8 согласно регламенту [cite: 26]
                if self.count >= 8:
                    self.finished = True
                    return True
        else:
            if self.is_tracking_object:
                self.frames_without_object += 1
                if self.frames_without_object > 10:
                    self.is_tracking_object = False
        return False

    def _is_yellow_in_view(self):
        img = self.camera.getImage()
        if not img: return False

        yellow_pixels = 0
        # Зона сканирования: центр по X, низ по Y (чтобы не видеть сквозь тонкие стенки)
        start_x, end_x = int(self.width * 0.4), int(self.width * 0.6)
        start_y, end_y = int(self.height * 0.45), int(self.height * 0.9)

        for x in range(start_x, end_x, 2): 
            for y in range(start_y, end_y, 2): 
                r = self.camera.imageGetRed(img, self.width, x, y)
                g = self.camera.imageGetGreen(img, self.width, x, y)
                b = self.camera.imageGetBlue(img, self.width, x, y)
                
                # --- УЛУЧШЕННАЯ ЛОГИКА ДЛЯ ГЛУБОКИХ ТЕНЕЙ ---
                # 1. Снижаем порог яркости до 25 (очень темно)
                # 2. Желтый в тени: R и G значительно больше B.
                # 3. Добавляем "+1" к B, чтобы избежать деления на ноль.
                
                is_bright_enough = (r > 25 and g > 25)
                is_yellow_ratio = (r / (b + 1) > 1.4) and (g / (b + 1) > 1.4)
                is_rg_balanced = abs(r - g) < 50 # R и G должны быть похожи
                
                if is_bright_enough and is_yellow_ratio and is_rg_balanced:
                    yellow_pixels += 1

        # Если нашли хотя бы 30 "подозрительных" пикселей в узком окне — это цилиндр
        return yellow_pixels > 30

    def print_final_message(self):
        # Вывод по регламенту: «Программа выполнена. Количество объектов - ХХ» 
        print(f"Программа выполнена. Количество объектов - {self.count:02d}")

# ============================================================
# 2) КАРТА (0 = проход, 1 = стена, 2..9 = целевые зоны)
# ============================================================
RAW_MAP = """
000000000000000000000000000000011000000000000000
000000000000000000022220000000011000000000000000
000000000000000000022220000000011000000000000000
000000000000000110022220000000011000000000000000
000000000000000110022220000000011000000000000000
000000000000000111111111111000011000000000000000
000000000000000111111111111000011333300000000000
000000000000000110000000000000011333300000000000
000000000000000110000000000000011333300000000000
000000000000000110000000000000011111111100000000
000000000000000110000000000000011111111100000000
000000000000000110000000000000011111111100000000
000000000000000110000000000000000000000000000000
000000000000000110000000000000000000000000000000
000000000000000110000000000000000000000000000000
000000000000000119999000000110000000000000000000
000000000000000119999000000110000000000000000000
000000000000000119999000000110000000000000000000
000000001111111111111000000111111111000000000000
000000001111111111111000000111111111000000000000
000000000000000000000000000110000000000000000000
000000000000000000000000000110000000001111111111
111111111111000000000000000110000000001144440000
111111111111000000000000000110000000001144440000
000000000011000000000000000110000000001144440000
000000000011000000000000000110000110001100000000
000000000011000000011111100110000110001100000000
000000000011000000011111100000000110001100000000
000000000011000000077771100000000110001100000000
000000000011000000077771100000000110001100000000
000000000011000000077771100000000110001100000000
000000088811000000077771100000000110001100000000
000000088811000000000001100000000110001100000000
000000088811000000000001100000000110001100000000
000000011111111000000001100000000110000000000000
000000011111111000000001100000000115550000000000
000000000000000000000001100000000115550000000000
000000000000000000000001100000000115550000000000
000000000000000000000001100000111111111111000000
000000000000000000000001166660111111111111000000
000000000000000000000001166660000000000000000000
000000000000000000000001166660000000000000000000
000000000000000000000001111100000000000000000000
000000000000000110000001111100000000000000000000
000000000000000110000000000000000000000000000000
000000000000000110000000000000000000000000000000
000000000000000110000000000000000000000000000000
""".strip()

# ============================================================
# 3) НАСТРОЙКИ РОБОТА / МИРА
# ============================================================
robot = Robot()
TS = int(robot.getBasicTimeStep())

WHEEL_RADIUS = 0.021
AXLE = 0.1054

CELL_SIZE = 0.0625     # !!! поставь реальный размер клетки мира

SPEED_FWD  = 5.0
SPEED_TURN = 3.0
KP_HEADING = 7.0

# ДЫРКИ
HOLE_THR = 20.0

# "ПОДЪЕЗД-ЗАГЛЯНУЛ-ОТЪЕХАЛ": насколько заезжать на цифровую клетку (метка на ней)
# если не хватает для детекта черного — увеличь (например до 0.03..0.05)
PEEK_DIST = 0.02        # 2 см
PEEK_SPEED = 1.8
PEEK_DWELL_STEPS = 30   # сколько шагов постоять на метке

# ============================================================
# 4) УСТРОЙСТВА
# ============================================================
lm = robot.getDevice("left wheel motor")
rm = robot.getDevice("right wheel motor")
lm.setPosition(float('inf'))
rm.setPosition(float('inf'))
lm.setVelocity(0.0)
rm.setVelocity(0.0)

le = robot.getDevice("left wheel sensor")
re = robot.getDevice("right wheel sensor")
le.enable(TS)
re.enable(TS)

gyro = robot.getDevice("gyro")
gyro.enable(TS)

g_l  = robot.getDevice("ground left infrared sensor")
g_r  = robot.getDevice("ground right infrared sensor")
g_fl = robot.getDevice("ground front left infrared sensor")
g_fr = robot.getDevice("ground front right infrared sensor")
for s in (g_l, g_r, g_fl, g_fr):
    s.enable(TS)

# ИНИЦ детектора цилиндров
scanner = CylinderScanner(robot, camera_name='camera')

robot.step(TS)

# ============================================================
# 5) ПАРСИНГ КАРТЫ + ЦЕЛИ
# ============================================================
GRID = [list(row) for row in RAW_MAP.splitlines()]
H = len(GRID)
W = len(GRID[0])

targets = defaultdict(list)  # digit -> [(x,y),...]
for y in range(H):
    for x in range(W):
        c = GRID[y][x]
        if c.isdigit() and c not in ('0', '1'):
            targets[int(c)].append((x, y))

target_digits = sorted([d for d in targets.keys() if 2 <= d <= 9 and d not in (4, 5)])

# ============================================================
# 6) ДВИЖЕНИЕ / ОРИЕНТАЦИЯ
# ============================================================
CURRENT_ANGLE = 0.0
TARGET_HEADING = 0.0

def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def stop():
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)

def step_and_update():
    """Единственная функция шага симуляции: тут и гироскоп, и цилиндры."""
    global CURRENT_ANGLE

    if robot.step(TS) == -1:
        return False

    # Гироскоп
    wz = gyro.getValues()[2]
    CURRENT_ANGLE += wz * (TS / 1000.0)

    # Детектор цилиндров
    scanner.update_and_check()

    return True

def is_hole_now():
    vals = [g_l.getValue(), g_r.getValue(), g_fl.getValue(), g_fr.getValue()]
    return any(v < HOLE_THR for v in vals)

def on_black_marker_now():
    # тот же порог и те же сенсоры, что внутри scanner (оставляем его логику),
    # но удобно проверять снаружи
    return (scanner.gs_l.getValue() < 500 or scanner.gs_r.getValue() < 500)

def soft_stop():
    for v in [2.0, 1.0, 0.5, 0.0]:
        lm.setVelocity(v)
        rm.setVelocity(v)
        if not step_and_update():
            break

def back_off_short():
    lm.setVelocity(-2.0)
    rm.setVelocity(-2.0)
    for _ in range(8):
        if not step_and_update():
            break
    soft_stop()

def turn_to_heading(heading):
    global TARGET_HEADING
    TARGET_HEADING = heading
    while True:
        if not step_and_update():
            return
        err = normalize_angle(TARGET_HEADING - CURRENT_ANGLE)
        if abs(err) < 0.015:
            break
        sp = max(0.8, min(SPEED_TURN, abs(err) * 6.0))
        if err > 0:
            lm.setVelocity(-sp)
            rm.setVelocity(+sp)
        else:
            lm.setVelocity(+sp)
            rm.setVelocity(-sp)
    stop()
    for _ in range(2):
        step_and_update()

def turn_left_90():
    global TARGET_HEADING
    TARGET_HEADING += math.pi/2
    turn_to_heading(TARGET_HEADING)

def turn_right_90():
    global TARGET_HEADING
    TARGET_HEADING -= math.pi/2
    turn_to_heading(TARGET_HEADING)

def drive_distance(dist_m, base_speed, stop_when_black=False):
    """
    Едем вдоль текущего TARGET_HEADING на dist_m (м).
    dist_m > 0 вперед, dist_m < 0 назад.
    """
    start_l = le.getValue()
    start_r = re.getValue()
    target_phi = abs(dist_m) / WHEEL_RADIUS
    direction = 1.0 if dist_m >= 0 else -1.0

    while True:
        if not step_and_update():
            return False

        if scanner.finished:
            return True

        # дырку не хотим ловить ни в каком режиме
        if is_hole_now():
            stop()
            for _ in range(2):
                step_and_update()
            back_off_short()
            return False

        # стоп по черной метке (для "заглянуть на цифру")
        if stop_when_black and on_black_marker_now():
            break

        dl = abs(le.getValue() - start_l)
        dr = abs(re.getValue() - start_r)
        if (dl + dr) / 2.0 >= target_phi:
            break

        err = normalize_angle(TARGET_HEADING - CURRENT_ANGLE)
        corr = KP_HEADING * err

        base = direction * base_speed
        vl = base - corr
        vr = base + corr
        vl = max(-12.0, min(12.0, vl))
        vr = max(-12.0, min(12.0, vr))

        lm.setVelocity(vl)
        rm.setVelocity(vr)

    soft_stop()
    return True


def move_one_cell_forward():
    return drive_distance(CELL_SIZE, SPEED_FWD, stop_when_black=False)

def peek_forward_and_back(max_dist=PEEK_DIST, speed=PEEK_SPEED, dwell_steps=PEEK_DWELL_STEPS):
    """
    Стоим в соседней клетке (0), смотрим на цифровую (2..9).
    Чуть-чуть заезжаем вперед, чтобы сенсоры зацепили черную метку на цифре,
    стоим несколько шагов для детектора, потом откатываемся назад ровно на пройденное.
    """
    # 1) вперед: но не дальше max_dist и можно остановиться раньше по черному
    start_l = le.getValue()
    start_r = re.getValue()

    ok = drive_distance(max_dist, speed, stop_when_black=True)
    if not ok:
        return False

    # сколько реально проехали вперед (в радианах колеса)
    fwd_phi = (abs(le.getValue() - start_l) + abs(re.getValue() - start_r)) / 2.0
    fwd_m = fwd_phi * WHEEL_RADIUS

    # 2) постоять на метке, дать scanner шанс увидеть желтый
    for _ in range(dwell_steps):
        if not step_and_update():
            return False
        if scanner.finished:
            return True

    # 3) назад ровно на то, что проехали вперед
    if fwd_m > 1e-4:
        ok = drive_distance(-fwd_m, speed, stop_when_black=False)
        if not ok:
            return False

    return True


# ============================================================
# 7) BFS ПО КАРТЕ + "НЕ ЗАЕЗЖАТЬ НА ЦИФРЫ"
# ============================================================
DIRS = [(1,0),(0,1),(-1,0),(0,-1)]  # E,S,W,N

def in_bounds(x, y):
    return 0 <= x < W and 0 <= y < H

def is_digit_zone(x, y):
    c = GRID[y][x]
    return c.isdigit() and c not in ('0', '1')  # 2..9

def passable(x, y, blocked):
    # запрещаем дырки, стены и ВСЕ цифровые зоны 2..9
    if (x, y) in blocked:
        return False
    if GRID[y][x] == '1':
        return False
    if is_digit_zone(x, y):
        return False
    return True

def bfs_path(start, goal_cells, blocked):
    sx, sy = start
    goals = set(goal_cells)
    if (sx, sy) in goals:
        return [(sx, sy)]

    q = deque([(sx, sy)])
    prev = {(sx, sy): None}

    while q:
        x, y = q.popleft()
        for dx, dy in DIRS:
            nx, ny = x + dx, y + dy
            if not in_bounds(nx, ny):
                continue
            if (nx, ny) in prev:
                continue
            if not passable(nx, ny, blocked):
                continue
            prev[(nx, ny)] = (x, y)
            if (nx, ny) in goals:
                path = [(nx, ny)]
                cur = (x, y)
                while cur is not None:
                    path.append(cur)
                    cur = prev[cur]
                path.reverse()
                return path
            q.append((nx, ny))
    return None

def dir_from_step(a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if (dx, dy) == (1, 0):  return 0
    if (dx, dy) == (0, 1):  return 1
    if (dx, dy) == (-1, 0): return 2
    if (dx, dy) == (0, -1): return 3
    return 0

def rotate_to_dir(cur_dir, need_dir):
    diff = (need_dir - cur_dir) % 4
    if diff == 0:
        return cur_dir
    if diff == 1:
        turn_right_90()
        return (cur_dir + 1) % 4
    if diff == 3:
        turn_left_90()
        return (cur_dir - 1) % 4
    if diff == 2:
        turn_right_90(); turn_right_90()
        return (cur_dir + 2) % 4
    return cur_dir

# ============================================================
# 8) СЧИТАЕМ "КЛЕТКИ ПОДЪЕЗДА" К ЗОНАМ 2..9 (только '0' вокруг)
# ============================================================
approach_cells = defaultdict(list)  # digit -> [(x,y),...]

for d, cells in targets.items():
    s = set()
    for (x, y) in cells:
        for dx, dy in DIRS:
            ax, ay = x + dx, y + dy
            if not in_bounds(ax, ay):
                continue
            # подъезжать можно только на чистый проход '0'
            if GRID[ay][ax] == '0':
                s.add((ax, ay))
    approach_cells[d] = list(s)


def get_adjacent_target_cell(d, cur_cell):
    cx, cy = cur_cell
    for (tx, ty) in targets[d]:
        if abs(tx - cx) + abs(ty - cy) == 1:
            return (tx, ty)
    return None

def scan_zone_without_entering(d, cur_cell, cur_dir):
    """
    Мы приехали ВПЛОТНУЮ к зоне (на клетку '0' рядом с цифрой),
    поворачиваемся лицом к цифре и делаем "peek".
    """
    tgt = get_adjacent_target_cell(d, cur_cell)
    if tgt is not None:
        need_dir = dir_from_step(cur_cell, tgt)
        cur_dir = rotate_to_dir(cur_dir, need_dir)

    before = scanner.count

    # попытка 1
    ok = peek_forward_and_back(PEEK_DIST, PEEK_SPEED, PEEK_DWELL_STEPS)
    if not ok:
        return cur_dir

    # если не детектнулось — попробуем чуть глубже (всё равно это "заглянуть", не ехать по цифре)
    if (scanner.count == before) and (not scanner.finished):
        ok = peek_forward_and_back(PEEK_DIST * 1.5, PEEK_SPEED, PEEK_DWELL_STEPS)
        if not ok:
            return cur_dir

    # чуть постоять после маневра
    stop()
    for _ in range(10):
        step_and_update()

    return cur_dir


# ============================================================
# 9) СТАРТ
# ============================================================
START_X, START_Y = 0, 0
cur_cell = (START_X, START_Y)
cur_dir = 0  # East
CURRENT_ANGLE = 0.0
TARGET_HEADING = 0.0

blocked_by_holes = set()

print(f"START cell={cur_cell}, dir=E, HxW={H}x{W}")
print(f"Targets digits found: {target_digits}")

# ============================================================
# 10) ОСНОВНАЯ ЛОГИКА:
#     - НЕ заезжаем на цифры 2..9 (BFS их считает стенами)
#     - подъезжаем на соседнюю '0'
#     - "заглядываем" на цифру на 1-2 см и отъезжаем назад
# ============================================================
for d in target_digits:
    if scanner.finished:
        break

    goal_cells = approach_cells[d]
    if not goal_cells:
        print(f"\n=== TARGET {d}: NO approach cells (no '0' around). Skip. ===")
        continue

    print(f"\n=== TARGET {d}: approach cells={len(goal_cells)} ===")

    while True:
        if scanner.finished:
            break

        path = bfs_path(cur_cell, goal_cells, blocked_by_holes)
        if not path:
            print(f"Нет пути к зоне {d} (к клеткам подъезда). Пропуск.")
            break

        reached = False

        for i in range(1, len(path)):
            if scanner.finished:
                reached = True
                break

            nxt = path[i]
            need_dir = dir_from_step(cur_cell, nxt)
            cur_dir = rotate_to_dir(cur_dir, need_dir)

            ok = move_one_cell_forward()
            if not ok:
                blocked_by_holes.add(nxt)
                print(f"HOLE/FAIL detected -> block cell {nxt} and replan...")
                reached = False
                break

             # ... (внутри цикла по target_digits)
            cur_cell = nxt

            if cur_cell in goal_cells:
                print(f"ARRIVED near target zone {d} at cell={cur_cell}")
                reached = True
                break

        if reached:
            # --- ВКЛЮЧАЕМ СКАНЕР ПЕРЕД ОСМОТРОМ ---
            scanner.active = True
            # Сбрасываем флаг слежения, чтобы гарантированно увидеть новый объект
            scanner.is_tracking_object = False 
            
            # Сканим зону, не заезжая на цифру (только "peek")
            cur_dir = scan_zone_without_entering(d, cur_cell, cur_dir)
            
            # --- ВЫКЛЮЧАЕМ СКАНЕР ПОСЛЕ ОСМОТРА ---
            scanner.active = False
            break
# ============================================================
# 11) ФИНИШ
# ============================================================
stop()
for _ in range(10):
    step_and_update()

scanner.print_final_message()
