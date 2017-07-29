function [pathL,goalf] = DijkstraForBoundary_mod_star(Graph, source, goal, value)
%DijkstraForBoundary_mod is a shortest distance function which returns pathL ` 
%Graph is a binary array, with 0 for every white, red, or blue square and
% 1 for everything else
% source is a list of the coordinates on Graph that are red (robots)
% Goal is a list of the coordinates of Graph that are blue (frontiers)
%  Authors: Arun Mahadev & Aaron T. Becker, based on
% https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
Graph=flipud(Graph);
Q = find(Graph==0); % create vertex set Q from the available spaces to navigate
dist = Inf*ones(size(Graph));
prevL = repmat('?',size(Graph));
prev = -1*ones(size(Graph));
path = []; %#ok<NASGU>
pathL = [];       
dist(source) = 0;  % Distance from source to source is zero
dist=flipud(dist);
goalmat=zeros(size(Graph));
goalmat(goal)=1;% the goals are where it's 1
goalmat=flipud(goalmat);
goal=find(goalmat);% Get the indice after you flip the matrix
% while Q is not empty:
mincost=100000000000;
level=0;
lowvalflag=0;
while numel(Q)>0
    
    [~,ind] = min(dist(Q)); % Source node will be selected first from the indices
                            % where the graph can be traversed
    dirs = [ 0,-1;% left
        -1,0; % up
        0,1;  % right
        1,0 ; ];  % down
    u = Q(ind);
    Q(ind) = []; % remove u from Q
    dirLetter = ['l','u','r','d'];
    [ui,uj] = ind2sub(size(Graph),u); % Get row column index of u
    for i = 1:size(dirs,1) % for each neighbor v of u: where v is still in Q.
                           % Navigates through each direction next the source
        v = sub2ind(size(Graph), dirs(i,1) + ui,  dirs(i,2) +uj);
        %Get the row column index of the target node to explore
        if Graph(v) == 0 %only try to move if the vertex is 0
            alt = 1+dist(u); %distance to node v if we come from node u
                             %Note that dist(u) is zero because it is the
                             %source node
            if alt < dist(v) %Compares it to infinity, will probably always
                             %be true
                dist(v) = alt;%set that distance
                prevL(v) = dirLetter(i);% get the move sequence
                prev(v) = u;% previous indice traversed is v
                % find if we have reached a goal node is v in goal.If so,
                % return path
               if any(goal==v)%If a frontier is our currently explored node
                   if dist(v)==1 && value(v)<=-2 %cost of -1 or -2
                       %If the cost of our explored cell is the
                       %minimum we are greedy and go directly to it!
                      lowvalflag=1;%Go get this node and its path!!
                   elseif dist(v)==1 %If it's just the closestFrontier
                       mincost=dist(v)+value(v);%This is our default minimum cost
                       tempmin=v;%We get our index of our minimum
                       break;%Hopefully go to this node and generate our path?
                   end
                   if dist(v)==2 %2 step exploration, our value of the cell
                                 %must be -3 to justify this 
                      if dist(v)+value(v)<=mincost %If it's better than the
                          %1 step exploration cost
                          lowvalflag=1;%We go to this node
                      else
                          break;
                      end
                   end
%                    if level==2
%                        if value(v)<=mincost-3
%                            lowvalflag=1;
%                        else
%                        v=tempmin;
%                        lowvalflag=1;
%                        break;
%                        end
%                    end 
%                   Not needed anymore
                end
        
               if lowvalflag==1%If it's our minimum
                   goalf = v;
                   path = zeros(1,dist(v));
                   pathL = repmat(' ',1,dist(v));
                   path(dist(v)) = prev(v);
                   pathL(dist(v)) = prevL(v);
                   for k = dist(v)-1:-1:1
                       path(k) = prev(path(k+1));
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
    goalf=find(goalbackmat); %finding location of goal frontier cell to be returned.

end