function [pathL,goalf] = DijkstraForBoundary_mod(Graph, source, goal)
% Graph is a binary array, with 0 for every white, red, or blue square and
% 1 for everything else
% Source is a list of the coordinates of Graph that are red (robots)
% Goal is a list of the coordinates of Graph that are blue (boundaries)
%  Authors: Arun Mahadev & Aaron T. Becker, based on
% https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
% 1  function Dijkstra(Graph, source):
%  2
%  3      create vertex set Q
%  5      for each vertex v in Graph:             // Initialization
%  6          dist[v] ? INFINITY                  // Unknown distance from source to v
%  7          prev[v] ? UNDEFINED                 // Previous node in optimal path from source
%  8          add v to Q                          // All nodes initially in Q (unvisited nodes)
Graph=flipud(Graph);
Q = find(Graph==0);
dist = Inf*ones(size(Graph));
prevL = repmat('?',size(Graph));
prev = -1*ones(size(Graph));
pathL = [];
%  9     for each vertex v in source:
% 10      dist[source] ? 0                        // Distance from source to source
dist(source) = 0;
dist=flipud(dist);
goalmat=zeros(size(Graph));
goalmat(goal)=1;
goalmat=flipud(goalmat);
goal=find(goalmat);
% 11
% 12      while Q is not empty:
while numel(Q)>0
    % 13          u ? vertex in Q with min dist[u]    // Source nodes will be selected first
    [~,ind] = min(dist(Q));
    
    % 16          for each neighbor v of u:           // where v is still in Q.
    % up, right, down, left
    dirs = [ -1,0;   0,1;  1,0;  0,-1];
    u = Q(ind);
    % 14          remove u from Q
    Q(ind) = [];
    dirLetter = ['u','r','d','l'];
    [ui,uj] = ind2sub(size(Graph),u);
    for i = 1:size(dirs,1)
        
        v = sub2ind( size(Graph), dirs(i,1) + ui,  dirs(i,2) +uj);
        if Graph(v) == 0 %only try to move if the vertex is 0
            alt = 1+dist(u); %distance to node v if we come from node u
            if alt < dist(v)
                dist(v) = alt;
                prevL(v) = dirLetter(i);
                prev(v) = u;
                % find if we have reached a goal node is v in goal?  if so,
                % return path
                if any(goal==v)
                    goalf=v;
                    pathL = repmat(' ',1,dist(v));
                    pathL(dist(v)) = prevL(v);
                    for k = dist(v)-1:-1:1
                        pathL(k) =prevL(path(k+1));
                    end
                    Q = [];
                    
                end
            end
        end
    end
end
goalbackmat=zeros(size(goalmat));
goalbackmat(goalf)=1;
goalbackmat=flipud(goalbackmat);
goalf=find(goalbackmat);

% 17              alt ? dist[u] + length(u, v)
% 18              if alt < dist[v]:               // A shorter path to v has been found
% 19                  dist[v] ? alt
% 20                  prev[v] ? u
end