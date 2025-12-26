import random
from dataclasses import dataclass, field
from typing import Optional, FrozenSet, Dict, Tuple, List, Set


@dataclass(order=True)
class Edge:
    u: str                              
    v: str                              
    distance: int = 1
    time: int = 1
    accessible: bool = True
    closed: bool = False
    line_id: Optional[int] = field(default=None, compare=False) 
    text_id: Optional[int] = field(default=None, compare=False)  
    
    
    def key(self) -> FrozenSet[str]:
        return frozenset({self.u, self.v})


    def color(self) -> str:
        if self.closed:
            return "red"
        if not self.accessible:
            return "orange"
        return "black"


class Graph:
    def __init__(self):
        self.nodes: Dict[str, Tuple[int, int, Optional[int], Optional[int]]] = {} 
        self.edges: Dict[FrozenSet[str], Edge] = {} 
        self.adj: Dict[str, Dict[str, Edge]] = {} 
     
        
    def add_node(self, name, x, y):  
        if name in self.nodes:
            raise ValueError(f"Duplicate node '{name}'")
        self.nodes[name] = (x, y, None, None)
        self.adj[name] = {}
      
        
    def add_edge(self, u, v, distance, time, accessible) -> Edge:
        if u == v:
            raise ValueError("Cannot connect a node to itself")
        if u not in self.nodes or v not in self.nodes:
            raise ValueError("Both endpoints must exist")      
        key = frozenset({u, v})
        if key in self.edges:
            raise ValueError("Edge already exists")
        e = Edge(u, v, distance, time, accessible)
        self.edges[key] = e
        self.adj[u][v] = e
        self.adj[v][u] = e
        return e
    
    
    def remove_edge(self, u, v) -> Edge:
        key = frozenset({u, v})
        e = self.edges.pop(key, None)
        if not e:
            raise ValueError("Edge does not exist")
        if u in self.adj and v in self.adj[u]:
            del self.adj[u][v]
        if v in self.adj and u in self.adj[v]:
            del self.adj[v][u]
        return e
    
    
    def randomize_edge_weights(self):
        for e in self.edges.values():
            e.distance = random.randint(1, 20)
            e.time = random.randint(1, 20)
    
    
    def get_edge(self, u, v):
        return self.edges.get(frozenset({u, v}))
    
    
    def neighbors(self, node, accessible_only):
        out = []
        for nbr, e in self.adj.get(node, {}).items():
            if e.closed: 
                continue
            if accessible_only and not e.accessible:
                continue
            out.append((nbr, e))
        out.sort(key=lambda t: t[0])
        return out


    def toggle_closed(self, u, v):
        e = self.get_edge(u, v)
        if not e:
            raise ValueError("Edge does not exist")
        e.closed = not e.closed
        return e
    
    
    def toggle_accessibility(self, u, v):
        e = self.get_edge(u, v)
        if not e:
            raise ValueError("Edge does not exist")
        e.accessible = not e.accessible
        return e
     
            
    def remove_node(self, name) -> List[Edge]:
        if name not in self.nodes:
            raise ValueError("Node does not exist")
        neighbors = list(self.adj[name].keys())
        removed_edges: List[Edge] = []
        for nbr in neighbors:
            e = self.get_edge(name, nbr)
            if e:
                self.remove_edge(name, nbr)
                removed_edges.append(e)
        del self.adj[name]
        del self.nodes[name]
        return removed_edges


    def to_dict(self):
        return {
            "nodes": [
                {"name": n, "x": data[0], "y": data[1]} 
                for n, data in self.nodes.items()
            ],
            "edges": [
                {
                    "u": e.u, "v": e.v, 
                    "distance": e.distance, "time": e.time, 
                    "accessible": e.accessible, "closed": e.closed
                }
                for e in self.edges.values()
            ]
        }


    def from_dict(self, data):
        self.nodes.clear()
        self.edges.clear()
        self.adj.clear()
        
        for n_data in data["nodes"]:
            self.nodes[n_data["name"]] = (n_data["x"], n_data["y"], None, None)
            self.adj[n_data["name"]] = {}
            
        for e_data in data["edges"]:
            self.add_edge(
                e_data["u"], e_data["v"], 
                e_data["distance"], e_data["time"], 
                e_data["accessible"]
            )
            if e_data.get("closed", False):
                self.edges[frozenset({e_data["u"], e_data["v"]})].closed = True


    def clear(self):
        self.nodes.clear()
        self.edges.clear()
        self.adj.clear()