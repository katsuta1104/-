def permutation(my_list):
    if len(my_list) == 1:
        return [my_list]
    else:
        result = []
        for i in range(len(my_list)):
            rest = permutation(my_list[:i] + my_list[i+1:])
            for rest_perm in rest:
                result.append([my_list[i]] + rest_perm)
        return result

def diside():
    # シャープが移動不可、ドットが移動可
    grid = [
        ['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'],
        ["#", ".", ".", ".", ".", "#", "#", "#", "#", ".", ".", ".", ".", ".", "#", "#", "#"],
        ["#", ".", ".", ".", ".", "#", "#", "#", "#", ".", ".", ".", ".", ".", "#", "#", "#"],
        ["#", "#", "#", "#", ".", "#", "#", "#", "#", ".", "#", "#", "#", ".", "#", "#", "#"],
        ["#", "#", "#", "#", ".", "#", "#", "#", "#", ".", "#", "#", "#", ".", "#", "#", "#"],
        ["#", "#", "#", "#", ".", "#", "#", "#", "#", ".", "#", "#", "#", ".", ".", ".", "#"],
        ["#", "#", "#", "#", ".", ".", ".", ".", ".", ".", ".", "#", "#", ".", ".", ".", "#"],
        ["#", ".", ".", ".", ".", "#", "#", ".", ".", ".", ".", ".", ".", ".", ".", ".", "#"],
        ["#", ".", ".", ".", ".", "#", "#", "#", ".", ".", ".", ".", ".", ".", ".", ".", "#"],
        ["#", "#", "#", ".", ".", "#", "#", "#", ".", ".", "#", "#", "#", ".", ".", ".", "#"],
        ["#", "#", "#", "#", ".", ".", ".", ".", ".", ".", "#", "#", "#", ".", ".", ".", "#"],
        ["#", "#", "#", "#", ".", ".", ".", ".", ".", ".", "#", "#", "#", ".", "#", "#", "#"],
        ["#", "#", "#", "#", "#", "#", ".", ".", ".", ".", ".", "#", ".", ".", "#", "#", "#"],
        ["#", ".", ".", "#", "#", "#", ".", ".", ".", ".", ".", ".", ".", ".", ".", "#", "#"],
        ["#", ".", ".", "#", "#", "#", ".", "#", "#", "#", "#", ".", ".", ".", ".", ".", "#"],
        ["#", ".", ".", ".", ".", ".", ".", "#", "#", "#", "#", ".", ".", ".", ".", ".", "#"],
        ['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#']
    ]
    target = [(7, 9), (14, 13), (11, 7), (15, 2), (8, 3), (1, 2), (1, 11), (8, 15)]
    return grid, target

def bfs(grid, start, goal):
    H, W = len(grid), len(grid[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 上下左右
    queue = [(start[0], start[1], 0)]  # (y, x, 距離)
    visited = set()
    visited.add(start)
    parent = {start: None}

    while queue:
        y, x, dist = queue.pop(0)  # pop(0) でキューの先頭を取り出す
        if (y, x) == goal:
            path = []
            current = goal
            while current is not None:
                path.append(current)
                current = parent[current]
            path.reverse()
            return dist, path

        for dy, dx in directions:
            ny, nx = y + dy, x + dx
            if 0 <= ny < H and 0 <= nx < W and (ny, nx) not in visited and grid[ny][nx] == ".":
                visited.add((ny, nx))
                parent[(ny, nx)] = (y, x)
                queue.append((ny, nx, dist + 1))  # 新しい位置をキューに追加

    # 到達不可能な場合
    print("達成不可能です！")
    exit()

def matrix(grid, target):
    N = len(target)
    dis = [[0] * N for _ in range(N)]
    for i in range(N):
        for j in range(i + 1, N):
            dist, _ = bfs(grid, target[i], target[j])
            dis[i][j] = dist
            dis[j][i] = dist
    return dis

def tsp(dis):
    N = len(dis)
    min_dist = float("inf")
    best_path = None
    for start in range(1, 2):  # 開始地点は0
        for perm in permutation([i for i in range(N) if i != start]):
            dist = 0
            current_path = [start] + perm + [start]
            for i in range(len(current_path) - 1):
                dist += dis[current_path[i]][current_path[i + 1]]

            if dist < min_dist:
                min_dist = dist
                best_path = current_path

    return min_dist, best_path

def generate(grid, target, path):
    current_direction = "U"
    direction = {(-1, 0): "U", (1, 0): "D", (0, -1): "L", (0, 1): "R"}
    move = ""

    for i in range(len(path) - 1):
        start = target[path[i]]
        goal = target[path[i + 1]]
        _, path_segment = bfs(grid, start, goal)
        for j in range(1, len(path_segment)):
            prev = path_segment[j - 1]
            curr = path_segment[j]
            step = (curr[0] - prev[0], curr[1] - prev[1])

            if step in direction:
                step_dire = direction[step]
                
                if step_dire == current_direction:
                    move += "S"
                else:
                    if (current_direction == 'U' and step_dire == 'R') or \
                       (current_direction == 'R' and step_dire == 'D') or \
                       (current_direction == 'D' and step_dire == 'L') or \
                       (current_direction == 'L' and step_dire == 'U'):
                        move += "R"
                    elif (current_direction == 'U' and step_dire == 'L') or \
                         (current_direction == 'R' and step_dire == 'U') or \
                         (current_direction == 'D' and step_dire == 'R') or \
                         (current_direction == 'L' and step_dire == 'D'):
                        move += "L"
                    else:
                        move += "LL"
                    current_direction = step_dire
                    move += "S"

        move += "W"
    return move

def codey_move(path):
    dist = 5.5  # 一マスあたりの移動距離
    speed = 1   # 一秒あたり進む距離(40％基準)
    i = 0
    while i < len(path):
        s = path[i]
        
        if s == "S":
            count = 0
            for j in range(i, len(path)):
                if path[j] == "S":
                    count += 1
                else:
                    break
            print("直進:", (speed * count) / dist)
            i += count - 1
        
        elif s == "L":
            print("反時計回りに回転")
        
        elif s == "R":
            print("時計回りに回転")
        
        else:
            print("待機")
        i += 1

def main():
    grid, target = diside()
    dist = matrix(grid, target)
    min_dist, best_path = tsp(dist)
    best_path = list(best_path)

    print("最短経路:", *best_path)
    print("距離:", min_dist)

    path = generate(grid, target, best_path)
    print(path)

    codey_move(path)

main()
