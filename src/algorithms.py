import heapq
import math
from collections import deque
from typing import List, Optional, Dict, Set
from .utils import DisjointSet


def bfs(graph, start, goal, accessible_only):
    if start not in graph.nodes or goal not in graph.nodes:
        raise ValueError("Start and goal must be valid nodes")
    q = deque([start])
    visited: Set[str] = {start}
    parent: Dict[str, Optional[str]] = {start: None}
    order: List[str] = []
    discover: List[str] = []
    
    while q:
        cur = q.popleft()
        order.append(cur)
        if cur == goal:
            break
        for number, _ in graph.neighbors(cur, accessible_only):
            if number not in visited:
                visited.add(number)
                parent[number] = cur
                q.append(number)
                discover.append(number)
                
    if goal not in parent:
        return [], order, discover
    
    path = []
    cur: Optional[str] = goal
    while cur is not None:
        path.append(cur)
        cur = parent[cur]
    path.reverse()
    return path, order, discover


def dfs(graph, start, goal, accessible_only):
    if start not in graph.nodes or goal not in graph.nodes:
        raise ValueError("Start and goal must be valid nodes")
    visited: Set[str] = set()
    parent: Dict[str, Optional[str]] = {start: None}
    found = False
    order: List[str] = [] 
    discover: List[str] = []
    
    
    def recurse(u):
        nonlocal found
        if found:
            return
        visited.add(u)
        order.append(u)
        if u == goal:
            found = True 
            return
        for number, _ in graph.neighbors(u, accessible_only):
            if number not in visited:
                parent[number] = u
                discover.append(number)
                recurse(number)
    
    recurse(start)
    if not found:
        return [], order, discover
    
    path = []
    cur: Optional[str] = goal
    while cur is not None:
        path.append(cur)
        cur = parent[cur]
    path.reverse()
    return path, order, discover


def heuristic(graph, u, v, weight_type): 
    x1, y1, _, _ = graph.nodes[u]
    x2, y2, _, _ = graph.nodes[v]
    pixel_dist = math.hypot(x2 - x1, y2 - y1) if weight_type == "distance" else 0
    return pixel_dist / 50.0


def dijkstra(graph, start, goal, weight_type, accessible_only):
    return _weighted_search(graph, start, goal, weight_type, accessible_only, use_heuristic=False)


def astar(graph, start, goal, weight_type, accessible_only):
    return _weighted_search(graph, start, goal, weight_type, accessible_only, use_heuristic=True)


def _weighted_search(graph, start, goal, weight_type, accessible_only, use_heuristic):
    if start not in graph.nodes or goal not in graph.nodes:
        raise ValueError("Start and goal must be valid nodes")

    pq = [(0, start)]        
    costs = {start: 0}
    parent = {start: None}        
    visited_order = []      
    discover_order = []     

    while pq:
        cur_priority, u = heapq.heappop(pq)
        if cur_priority > costs.get(u, float('inf')) + (heuristic(graph, u, goal, weight_type) if use_heuristic else 0):
            continue
        visited_order.append(u)
        if u == goal:
            break

        for v, edge in graph.neighbors(u, accessible_only):
            weight = edge.distance if weight_type == "distance" else edge.time
            new_cost = costs[u] + weight

            if new_cost < costs.get(v, float('inf')):
                costs[v] = new_cost
                parent[v] = u
                priority = new_cost 

                if use_heuristic:
                    priority += heuristic(graph, v, goal, weight_type)                    
                heapq.heappush(pq, (priority, v))
                discover_order.append(v)

    if goal not in parent:
        return [], visited_order, discover_order        
    path = []
    curr = goal
    while curr is not None:
        path.append(curr)
        curr = parent[curr]
    path.reverse()
    return path, visited_order, discover_order


def kruskal(graph, weight_type, accessible_only):
    valid_edges = []
    for e in graph.edges.values():
        if e.closed: continue
        if accessible_only and not e.accessible: continue
        weight = e.distance if weight_type == "distance" else e.time
        valid_edges.append((weight, e))
        
    valid_edges.sort(key=lambda x: x[0])
    dsu = DisjointSet(graph.nodes.keys())
    mst_edges = []
    total_cost = 0
    
    for weight, e in valid_edges:
        if dsu.union(e.u, e.v):
            mst_edges.append(e)
            total_cost += weight
            
    return mst_edges, total_cost


def prim(graph, start_node, weight_type, accessible_only):
    if start_node not in graph.nodes:
        raise ValueError("Start node required for Prim's")
        
    mst_edges = []
    visited = {start_node}
    total_cost = 0
    
    pq = []
    for nbr, e in graph.neighbors(start_node, accessible_only):
        w = e.distance if weight_type == "distance" else e.time
        heapq.heappush(pq, (w, nbr, e))
        
    while pq:
        weight, u, edge = heapq.heappop(pq)
        
        if u in visited:
            continue
            
        visited.add(u)
        mst_edges.append(edge)
        total_cost += weight
        
        for nbr, e in graph.neighbors(u, accessible_only):
            if nbr not in visited:
                w = e.distance if weight_type == "distance" else e.time
                heapq.heappush(pq, (w, nbr, e))
                
    return mst_edges, total_cost