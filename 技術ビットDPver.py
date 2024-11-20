#import codey,rocky,time,event

class Queue(object):
    def __init__(self):
        self.s1 = []
        self.s2 = []
    
    def push(self,x):
        while self.s1:
            self.s2.append(self.s1.pop())
        self.s1.append(x)
        while self.s2:
            self.s1.append(self.s2.pop())
    
    def popleft(self):
        return self.s1.pop()
    
    def front(self):
        return self.s1[-1]

def diside():
    #シャープが移動不可、ドットが移動可
    grid=[
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
    #for g in grid:print("".join(g))
    #0-indexed
    target = [(7,9),(14,13),(11,7),(15,2),(8,3),(1,2),(1,11),(8,15)]

    return grid,target


def bfs(grid,start,goal):
    #BFS(幅優先探索)を用いることで最短距離を計算する
    
    H,W = len(grid),len(grid[0])
    directions = [(-1,0),(1,0),(0,-1),(0,1)]
    directions_names = ["U","D","L","P"]
    queue = Queue()
    queue.push((start[0],start[1],0))
    
    visited = set()
    visited.add(start)
    parent = {start:None}
    
    while queue:
        y,x,dist = queue.popleft()
        if (y,x) == goal:
            path = []
            current = goal
            while current is not None:
                path.append(current)
                current = parent[current]
            path.reverse()
            return dist,path
        
        for dy,dx in directions:
            ny,nx = y+dy,x+dx
            if 0<=ny<H and 0<=nx<W and (ny,nx) not in visited and grid[ny][nx]==".":
                visited.add((ny,nx))
                parent[(ny,nx)] = (y,x)
                queue.push((ny,nx,dist+1))
                
    #到達できないのでエラーを出力する
    exit(print("達成不可能です！"))

def matrix(grid,target):
    N=len(target)
    dis = [[0]*N for _ in range(N)]
    
    for i in range(N):
        for j in range(i+1,N):
            dist,_ = bfs(grid,target[i],target[j])
            dis[i][j] = dist
            dis[j][i] = dist
            
    return dis




def tsp(dis):
    N = len(dis)
    INF = float("inf")
    
    
    dp = [[INF] * N for _ in range(1 << N)]
    parent = [[-1] * N for _ in range(1 << N)]  # 経路復元用
    
    # 初期状態
    dp[1][0] = 0  # 開始地点は0

    # 状態遷移
    for mask in range(1 << N):
        for i in range(N):
            if not (mask & (1 << i)):
                continue

            for j in range(N):
                if mask & (1 << j): 
                    continue

                new_mask = mask | (1 << j)
                new_cost = dp[mask][i] + dis[i][j]
                if dp[new_mask][j] > new_cost:
                    dp[new_mask][j] = new_cost
                    parent[new_mask][j] = i

    # 最小コスト計算
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


def diside_start(n,path):#nをスタートした形に調整する
    while path[0] != n:
        path.append(path[0])
        del path[0]
    
    path.append(1)
    return path

def generate(grid,target,path):
    #具体的なpathを構築する
    #S:直進,L:反時計回りに90°回転,R:時計回りに90°回転,Wが頂点に到着した時に待つ

    current_direction = "U"
    direction = {(-1,0):"U",(1,0):"D",(0,-1):"L",(0,1):"R"}
    move = ""
    
    for i in range(len(path)-1):
        start = target[path[i]]
        goal = target[path[i+1]]
        _,path_segment = bfs(grid,start,goal)
        for j in range(1,len(path_segment)):
            prev = path_segment[j-1]
            curr = path_segment[j]
            step = (curr[0]-prev[0],curr[1]-prev[1])
            #print(step,curr,goal,start)
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
                        move+="LL"
                    current_direction = step_dire
                    move += "S"
                    
        move += "W"
    return move
                           
    


def codey_move(path):
    dist=5.5 #一マスあたりの移動距離
    speed=1 #一秒あたり何cm進むのか(40％基準)
    i = 0
    while i < len(path):
        s=path[i]
        
        if s == "S":
            count = 0
            for j in range(i,len(path)):
                if path[j] == "S":
                    count +=1
                else:
                    break
            
            print("直進:",(speed*count)/dist)
            i += count-1
        
        elif s == "L":
            print("反時計回りに回転")
        
        elif s == "R":
            print("時計回りに回転")
        
        else:
            print("待機")
        i+=1

def main():
    grid,target = diside()
    dist = matrix(grid,target)
    min_dist,best_path = tsp(dist)
    best_path = list(best_path)
    best_path = diside_start(1,best_path)#pathの順番を調整
    #最短経路探索終了
    
    print("最短経路:",*best_path)
    print("距離:",min_dist)

    path = generate(grid,target,best_path)
    print(path)

    
    codey_move(path)

main()
#ビットDPによって巡回セールスマン問題を解決する。O(2^N*N^2)
