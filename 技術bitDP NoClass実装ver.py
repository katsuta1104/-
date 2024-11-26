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
    # 目標地点の座標
    target = [(7,9),(14,13),(11,7),(15,2),(8,3),(1,2),(1,11),(8,15)]
    return grid, target

def bfs(grid, start, goal):
    # BFS(幅優先探索)で最短経路を計算する
    H, W = len(grid), len(grid[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 上下左右の方向
    queue = [(start[0], start[1], 0)]  # (y, x, 距離)
    visited = set()
    visited.add(start)
    parent = {start: None}
    
    while queue:
        y, x, dist = queue.pop(0)  # pop(0)でキューの最初の要素を取り出す
        if (y, x) == goal:
            # ゴールに到達したら経路を復元して戻す
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
                queue.append((ny, nx, dist + 1))  # 新しいノードをキューに追加
                
    # 到達不可能な場合はエラーメッセージ
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
    INF = float("inf")
    
    # dp[mask][i] = maskの状態でiにいる最短距離
    dp = [[INF] * N for _ in range(1 << N)]
    parent = [[-1] * N for _ in range(1 << N)]  # 経路復元用
    
    dp[1][0] = 0  # 開始地点は0（最初は0のみ訪問）

    for mask in range(1 << N):
        for i in range(N):
            if not (mask & (1 << i)):  # iがmaskに含まれていない場合
                continue
            for j in range(N):
                if mask & (1 << j):  # jがすでにmaskに含まれている場合
                    continue
                new_mask = mask | (1 << j)
                new_cost = dp[mask][i] + dis[i][j]
                if dp[new_mask][j] > new_cost:
                    dp[new_mask][j] = new_cost
                    parent[new_mask][j] = i

    # 最小コストの計算
    final_mask = (1 << N) - 1
    min_dist = INF
    last_node = -1

    for i in range(1, N):
        cost_with_return = dp[final_mask][i] + dis[i][0]
        if cost_with_return < min_dist:
            min_dist = cost_with_return
            last_node = i

    # 経路復元
    best_path = []
    mask = final_mask
    while last_node != -1:
        best_path.append(last_node)
        prev_node = parent[mask][last_node]
        mask &= ~(1 << last_node)
        last_node = prev_node
    
    best_path.reverse()
    return min_dist, best_path

def diside_start(n, path):  # nをスタートした形に調整する
    while path[0] != n:
        path.append(path[0])
        del path[0]
    
    path.append(1)
    return path

def generate(grid, target, path):
    # 具体的なpathを構築する
    direction = {(-1, 0): "U", (1, 0): "D", (0, -1): "L", (0, 1): "R"}
    current_direction = "U"
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
    speed = 1   # 一秒あたり進む距離(40%基準)
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
    best_path = diside_start(1, best_path)  # pathの順番を調整
    # 最短経路探索終了
    
    print("最短経路:", *best_path)
    print("距離:", min_dist)

    path = generate(grid, target, best_path)
    print(path)

    codey_move(path)

main()
