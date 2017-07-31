function [pathL,goalf] = BFS_Expansion( input_args )
%BFS_Expansion BFS of free spaces searching for frontier spaces along
%levels
%   Approach:
%   While frontier not found
%   Expand w/ dir Matrix -> delete u, add four dir nodes if possible
%       If frontier is found
%           Record address&value
%           If minimum
%               Break; compute path!
%Flag
%   Compute necessary expansions based on frontier values
%Loop every level of expansion
%   If it's a better frontier
%       Break; compute path
Graph=flipud(Graph);
Q = find(Graph==0); % create vertex set Q from the available free spaces
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
% Source node will be selected first from the indices

while numel(source)>0%While there are robots to explore
        %We make our direction matrix                
        dirs = [ 0,-1;% left
        -1,0; % up
        0,1;  % right
        1,0;];  % down
    u = Q(source(1));%We select a u to expand and take its
    %index
    Q(source(1)) = []; % remove u from Q
    source(1)=[];
    dirLetter = ['l','u','r','d'];
    [ui,uj] = ind2sub(size(Graph),u); % Get row column index of u
    frontierFound=0;
    frontiers=zeros(10000); %Pre-allocated array of frontiers and t
                                  %values for address evaluation
    while frontierFound==0%Logic might need help here
        for i = 1:size(dirs,1)%For each direction
            v = sub2ind(size(Graph), dirs(i,1) + ui,  dirs(i,2) +uj);
            %We get the cell in the direction
            if Graph(v) == 0 %only try to move if the vertex is 0
                alt = 1+dist(u); %distance to node v if we come from node u
                %Note that dist(u) is zero because it is the
                %source node
                if alt<dist(v)%If it's less than the current distance
                    %We store it in dist
                    dist(v)=alt;
                    prevL(v)=dirLetter(i);
                    prev(v)=u;
                    if any(goal==v)%If there's a frontier
                        if value(v)<=-2
                            frontierFound=1;
                            break;
                        end
                        frontiers(v)=value(v);
                    end
                end
            end
            
        end
    end
end
end

