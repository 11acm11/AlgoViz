import queue
import sys
import pygame

UI=pygame.display.set_mode((800,800))
pygame.display.set_caption("Robot Finding The Shortest Path")
class Node:
    def __init__(self,row,col,width,trows):
        self.row=row
        self.col=col
        self.x=row*width
        self.y=col * width
        self.color = (255,255,255)
        self.neighbors = []
        self.width =width
        self.trows=trows

    # Setting colour scheme for grid,path and nodes
    def get_pos(self):
        return self.row,self.col
    def closed(self):
        return self.color==(255,255,0)
    def barrier(self):
        return self.color==(0,0,0)
    def start(self):
        return self.color==(255,0,0)
    def goal(self):
        return self.color==(128,0,128)
    def reset(self):
        self.color=(255,255,255)

    def mclosed(self):
        self.color=(255,165,0)
    def mbarrier(self):
        self.color=(0,0,0)
    def mgoal(self):
        self.color=(128,0,128)
    def mpath(self):
        self.color=(64,224,208)
    def mstart(self):
        self.color=(255,0,0)   

    def draw(self,win):
        pygame.draw.rect(win,self.color,(self.x,self.y,self.width,self.width))
    
    # Adding neighbours for each node
    def ud_neighbors(self,grid):
        self.neighbours=[]
        if self.row < self.trows-1 and not grid[self.row+1][self.col].barrier():# down neighbour
            self.neighbours.append(grid[self.row+1][self.col])
        if self.row > 0 and not grid[self.row-1][self.col].barrier():# up neighbour
            self.neighbours.append(grid[self.row-1][self.col])
        if self.col < self.trows-1 and not grid[self.row][self.col+1].barrier():# right neighbour
            self.neighbours.append(grid[self.row][self.col+1])
        if self.row > 0 and not grid[self.row][self.col-1].barrier():# left neighbour
            self.neighbours.append(grid[self.row][self.col-1])

    def __lt__(self,other):
        return False
    
# Heuristic(Admissible) used for A* and Best First Search using Manhattan Distance
def heurist(p1,p2):
    x1,y1=p1
    x2,y2=p2
    return abs(x1-x2)+abs(y1-y2)


def new_path(prev,current, draw):
    while current in prev:
        current = prev[current]
        current.mpath()
        draw()


def astar(draw,grid,start,goal):
    count = 0
    # Priority Queue to pop the best node with lowest F score
    front = queue.PriorityQueue()
    front.put((0,count,start))
    prev = {} 
    visited={start}

    # G score : Distance between current node and starting node
    score_g={node:float("inf") for row in grid for node in row} # Holds G score for every node
    score_g[start]=0
    # F score : G score + Heuristic
    score_f={node:float("inf") for row in grid for node in row} # Holds F score for every node
    score_f[start]=heurist(start.get_pos(),goal.get_pos())
    
    while not front.empty():
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                pygame.quit()

        current = front.get()[2] # pops lowest f score node

        if current == goal:
            new_path(prev,goal,draw)
            goal.mgoal()
            start.mstart()
            return True
        
        for neighbour in current.neighbours:
            temp_g=score_g[current]+1 # Next G score for neighbours , node weight taken as 1

            if temp_g < score_g[neighbour]: # If another neighbour has lower cost 
                # Update values
                prev[neighbour]=current
                score_g[neighbour]=temp_g
                score_f[neighbour]=temp_g+heurist(neighbour.get_pos(),goal.get_pos())

                if neighbour not in visited:
                    count+=1
                    front.put((score_f[neighbour],count,neighbour))
                    visited.add(neighbour)
             
        draw()
        if current!=start:
            current.mclosed()
    return False


def bfs(draw,grid,start,goal):
    # Uses FIFO logic 
    front=[]
    front.append(start)
    prev={}
    visited = {}
    visited={start}

    while(len(front)>0):
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                pygame.quit()

        current=front.pop(0)# pops first element of queue

        if current == goal:
            new_path(prev,goal,draw)
            goal.mgoal()
            start.mstart()
            return True

        for neighbor in current.neighbours:
            if(neighbor not in visited):
                prev[neighbor]=current
                visited.add(neighbor)
                front.append(neighbor)
            
        draw()
        if current != start:
            current.mclosed()
    return False


def dfs(draw,grid,start,goal):
    # Uses LIFO logic
    front = []
    front= [start]
    prev = {}
    visited = {start}

    while front:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    return False

        current = front.pop() # pops top element from stack

        if current == goal:
            new_path(prev, goal, draw)
            goal.mgoal()
            start.mstart()
            return True

        for neighbour in current.neighbours:
            if neighbour not in visited:
                prev[neighbour] = current
                front.append(neighbour)
                visited.add(neighbour)

        draw()
        if current != start:
             current.mclosed()

    return False


def best(draw,grid,start,goal):
    count = 0
    # Priority queue to pop node with lowest F score
    front = queue.PriorityQueue()
    front.put((0, count, start))
    prev = {}
    visited = {start}

    # F score = Heuristic
    f_score = {Node: float("inf") for row in grid for Node in row}
    f_score[start] = heurist(start.get_pos(), goal.get_pos())

    while not front.empty():
        draw()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current_node = front.get()[2]# lowest F score node popped

        if current_node == goal:
            new_path(prev,goal,draw)
            goal.mgoal()
            start.mstart()
            return True

        for neighbour in current_node.neighbours:
            f_score[neighbour] = heurist(neighbour.get_pos(), goal.get_pos()) + 1# F score of neighbour

            if neighbour not in visited:
                prev[neighbour] = current_node
                visited.add(neighbour)
                count += 1
                front.put((f_score[neighbour], count, neighbour))

        if current_node not in (start, goal):
            current_node.mclosed()

    return False


def dijkstras(draw,grid,start,goal):
    count = 0
    # Priority Queue to pop node with lowest F score
    front = queue.PriorityQueue()
    front.put((0, count, start))
    prev = {}
    visited = {start}

    # F score = distance between current node and starting node
    f_score = {Node: float("inf") for row in grid for Node in row}# F score of every node
    f_score[start] = 0

    while not front.empty():
        draw()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current_node = front.get()[2]# pops node of lowest F score

        if current_node == goal:
            new_path(prev,goal,draw)
            goal.mgoal()
            start.mstart()
            return True

        next_f_score = f_score[current_node] + 1# Next F score for neighbours

        for neighbour in current_node.neighbours:
            if next_f_score < f_score[neighbour]:
                f_score[neighbour] = next_f_score + 1

                if neighbour not in visited:
                    prev[neighbour] = current_node
                    visited.add(neighbour)
                    count += 1
                    front.put((f_score[neighbour], count, neighbour))

        if current_node not in (start, goal):
            current_node.mclosed()
        
    return False


def bidirectional(draw,grid,start,goal):
    # Using LIFO logic
    front = queue.Queue()
    front.put(start)
    back = queue.Queue()
    back.put(goal)
    prev_start = {}
    prev_goal = {}
    visited_start={start}
    visited_goal={goal}

    while not front.empty() and not back.empty():
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                pygame.quit()

        # Pops first element of queues
        current_front=front.get()
        current_back=back.get()

        for neighbour_front,neighbour_back in zip(current_front.neighbours,current_back.neighbours):
            # Checks if front node visited
            if neighbour_front not in visited_start:
                visited_start.add(neighbour_front)
                prev_start[neighbour_front]=current_front
                front.put(neighbour_front)
            
            # Checks if back node visited 
            if neighbour_back not in visited_goal:
                visited_goal.add(neighbour_back)
                prev_goal[neighbour_back]=current_back
                back.put(neighbour_back)
            
            # If front node present in visited of back nodes
            if current_front in visited_goal:
                new_path(prev_start,current_front,draw)
                new_path(prev_goal,current_front,draw)
                current_front.mpath()
                goal.mgoal()
                start.mstart()
                return True
            # Or if back node prensent in visited of front nodes
            elif current_back in visited_start:
                new_path(prev_goal,current_back,draw)
                new_path(prev_start,current_back,draw)
                current_back.mpath()
                goal.mgoal()
                start.mstart()
                return True
        
        draw()
        if current_front != start and current_back!=goal:
            current_back.mclosed()
            current_front.mclosed()
    
    return False
                

def mgrid(rows,width):
    grid=[] 
    gap=width//rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node=Node(i,j,gap,rows)
            grid[i].append(node)
    return grid

def draw_grid(win,grid,rows,width):
    win.fill((255,255,255))
    for row in grid:
        for node in row:
            node.draw(win)
    gap=width//rows
    for i in range(rows):
        pygame.draw.line(win,(128,128,128),(0,i*gap),(width,i*gap))
        for j in range(rows):
            pygame.draw.line(win,(128,128,128),(j*gap,0),(j*gap,width))
    pygame.display.update()


def get_clicked_pos(pos,rows,width):
    gap=width//rows
    y,x=pos         
    row=y//gap
    col=x//gap
    return row,col
    

def main(win,width):
    Rows=50
    grid=mgrid(Rows,width)
    start=None
    goal=None
    run=True
    started=False
    while run:
        draw_grid(win,grid,Rows,width)
        for event in pygame.event.get():

            pos=pygame.mouse.get_pos()
            row,col=get_clicked_pos(pos,Rows,width)
            node=grid[row][col]
            
            if event.type==pygame.QUIT:
                run=False
                sys.exit()

            if pygame.mouse.get_pressed()[0]:
                if not start and node!=goal:
                    start=node
                    start.mstart()
                elif not goal and node!=start:
                    goal=node
                    goal.mgoal()
                elif node!=goal and node!=start:
                    node.mbarrier()
            elif pygame.mouse.get_pressed()[2]:
                node.reset()
                if node==start:
                    start=None
                elif node==goal:
                    goal=None

            if event.type==pygame.KEYDOWN:

                if event.key==pygame.K_1 and start and goal:
                    for row in grid:
                        for node in row:
                            node.ud_neighbors(grid)
                    astar(lambda: draw_grid(win,grid,Rows,width),grid,start,goal)

                if event.key==pygame.K_2 and start and goal:
                    for row in grid:
                        for node in row:
                            node.ud_neighbors(grid)
                    bfs(lambda: draw_grid(win,grid,Rows,width),grid,start,goal)

                if event.key==pygame.K_3 and start and goal:
                    for row in grid:
                        for node in row:
                            node.ud_neighbors(grid)
                    dfs(lambda:draw_grid(win,grid,Rows,width),grid,start,goal)

                if event.key==pygame.K_4 and start and goal:
                    for row in grid:
                        for node in row:
                            node.ud_neighbors(grid)
                    best(lambda:draw_grid(win,grid,Rows,width),grid,start,goal)

                if event.key==pygame.K_5 and start and goal:
                    for row in grid:
                        for node in row:
                            node.ud_neighbors(grid)
                    dijkstras(lambda:draw_grid(win,grid,Rows,width),grid,start,goal)

                if event.key==pygame.K_6 and start and goal:
                    for row in grid:
                        for node in row:
                            node.ud_neighbors(grid)
                    bidirectional(lambda:draw_grid(win,grid,Rows,width),grid,start,goal)

                if event.key==pygame.K_c:
                    start=None
                    goal=None
                    grid=mgrid(Rows,width)
        
    pygame.QUIT
main(UI,800)