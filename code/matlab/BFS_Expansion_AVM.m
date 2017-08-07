function [pathL,goalf] = BFS_Expansion_AVM( Graph, source, goal, value )
%BFS_Expansion BFS of free spaces searching for frontier spaces along
%levels
%   Approach:
%   While frontier not found
%   Expand w/ dir Matrix -> delete u, add four dir nodes if possible
%       If frontier is found
%           Record address&value
%           If minimum
%               Break; compute path!
%Flag(when the minimum isn't found)
%   Compute necessary expansions based on frontier values
%Loop every level of expansion
%   If it's a better frontier
%       Break; compute path
% Graph=flipud(Graph);
%  value=flipud(value);
prevL = repmat('?',size(Graph));
prev = -1*ones(size(Graph));
path = []; %#ok<NASGU>
pathL = [];
% goalmat=zeros(size(Graph));
% goalmat(goal)=1;% the goals are where it's 1
% goalmat=flipud(goalmat);
% goal=find(goalmat);% Get the indice after you flip the matrix
% while Q is not empty:
mincost=100000000000;
level=0;
lowvalflag=0;
dist=0;%Distance from the source node we have expanded from
level_zero=0;%If a frontier has been found
PassSource=source;%Storage variable for source
% Source node will be selected first from the indices
while lowvalflag==0%While we haven't found a good frontier
    %And there has been no minimum visited
    %We make our direction matrix
    dirs = [ 0,-1;% left
        -1,0; % up
        0,1;  % right
        1,0;];  % down
    
    k=0;
    %values for address evaluation
    while numel(source)>0%Free space loop
        u=source(1);%We take and remove the first source node
        source(1)=[];
        [ui,uj] = ind2sub(size(Graph),u); % Get row column index of u
        for i = 1:size(dirs,1)%For each direction
            if dirs(i,1) + ui>=1 && dirs(i,2) +uj >=1 && dirs(i,1) +ui <= size(Graph,1) && dirs(i,2) +uj <= size(Graph,2)
                v = sub2ind(size(Graph), dirs(i,1) + ui,  dirs(i,2) +uj);
                %We get the cell in the direction and make sure it's within
                %the boundaries
                if Graph(v) == 0 %only try to move if the vertex is 0
                    if isempty(find(source==v))%If our explored free space
                        %isn't a robot location
                        k=k+1;%increment our new source indice
                        nextsource(k)=v;%store v
                        if numel(find(nextsource==v))>1%If there's more than
                            %1 v we have stored we need to remove it for
                            %optimization purposes
                            nextsource(k)=[];
                            k=k-1;%We decrement for continuity
                        end
                    end
                    %                     prevL(v)=dirLetter(i);
                    %                     prev(v)=u;
                    
                    if any(goal==v)%If there's a frontier
                        level_zero=1;
                        if value(v)<=-2 && level==0%If it's the best frontier
                            lowvalflag=1;%lowvalflage allows us to skip
                            mincost=value(v);%We found mincost, so we store it
                            bestfrontierloc=v;
                            %frontier evaluation and go to
%                             break;
                        elseif value(v)< mincost%If a frontier has a lower value
                            mincost=value(v);%We store it
                            if level==0%If it's the lowest level, it's the "preferred"
                                %frontier we want to go to because we're
                                %still greedy in some sorts
                                bestfrontierloc=v;
                            end
                        end
                    end
                end
            end
        end
%         dist=dist+1;%Increase the distance counter after an expansion
        if lowvalflag==1%If we found the best frontier we want to break out
            %of the while loop.
            break;
        end
    end
    if lowvalflag==0&&level_zero==1%If we didn't find a minimum and we found a frontier
        if level==0%If we're at our original level of exploration
            if mincost==0%And we found no valuable frontiers
                gracelvl=2;%We explore 2 more levels to maybe find a frontier
                %with a value of -3
            elseif mincost==-1%If we found a semi-valuable but not the best
                %Frontier, we explore another level to possible find a
                %frontier with value of -3
                gracelvl=1;
            end
        elseif level==1%Our first level of exploration!
            if gracelvl==1%If we need to explore one extra level
                if value(v)<mincost-1%If our value is less than our minimum cost
                    %plus the distance it took to reach it;aka it's a better frontier
                    bestfrontierloc=v;%We store it in bestfrontierloc
%                     dist=dist-1;%Lower dist to reflect an estimate of total cost
               
                end
                lowvalflag=1;%Trigger lowval flag so we end the loop once 
                %this level of exploration is finished
               
            end
        elseif level==2%Second level of exploration
            %Not sure why we're not checking for grace level;
            if value(v)<mincost-2%If it's a better frontier
                bestfrontierloc=v;%We store it!
                dist=dist-1;%
            else
                dist=dist-3;%
            end
            
            lowvalflag=1;%Trigger lowval flag so we end the loop once 
            %this level of exploration is finished
        end
            level=level+1;%
        
        
    end
    source=nextsource;%We cycle into the sources we've just expanded to
end
[frontdist,goalr, prevL,prev]= pathcalc(Graph,PassSource,bestfrontierloc);
% % %Now we calculate the path required to get to the frontier
%There seems to be a problem with the path calculated and how it's being
%located; it may be with the flipud thing we do to all of the cells but I'm
%not sure either
path = zeros(1,frontdist);
pathL = repmat(' ',1,frontdist);
path(frontdist) = prev(goalr);
pathL(frontdist) = prevL(goalr);
for l = frontdist-1:-1:1
    path(l) = prev(path(l+1));
    pathL(l) =prevL(path(l+1));
end
pathL=fliplr(pathL);
goalbackmat=zeros(size(Graph));
goalbackmat(bestfrontierloc)=1;
goalbackmat=flipud(goalbackmat);
goalf=find(goalbackmat); %finding location of goal frontier cell to be returned.
end

function [dist,goalf, prevL,prev]= pathcalc(Graph,RobotAddr,Frontier)
% The point of this code is to do a reverse BFS and find the robot from the
% best frontier we located.
% Graph=flipud(Graph);
% Robomat=zeros(size(Graph));
% Robomat(RobotAddr)=1;
% Robomat=flipud(Robomat);
% RobotAddr=find(Robomat);
source=Frontier;%The frontier cell we start from
prevL = repmat('?',size(Graph));%We make an array of move letters
prev = -1*ones(size(Graph));%We instantiate this with negative ones, but it
% stores the previous source we have come from
path = []; %#ok<NASGU>
RobotFound=0;%Flag for breaking out of the loops
dist=0;
dirs = [ 0,-1;% left
    -1,0; % up
    0,1;  % right
    1,0;];  % down
dirLetter = ['r','u','l','d'];
while RobotFound==0%We stop when we find the robot
    k=0;
    while numel(source)>0%We still need to explore through all of the free
        % spaces and use the updated sources
        u=source(1);
        source(1)=[];
        [ui,uj] = ind2sub(size(Graph),u); % Get row column index of u
        for i = 1:size(dirs,1)%For each direction
            if dirs(i,1) + ui>=1 && dirs(i,2) +uj >=1 && dirs(i,1) +ui <= size(Graph,1) && dirs(i,2) +uj <= size(Graph,2)
                v = sub2ind(size(Graph), dirs(i,1) + ui,  dirs(i,2) +uj);
                %We get the cell in the direction and check if it's valid
                if Graph(v) == 0 %only try to move if the vertex is 0/free space
                    if isempty(find(source==v))%We do the same thing from before
                        k=k+1;
                        nextsource(k)=v;
                        if numel(find(nextsource==v))>1
                            nextsource(k)=[];
                            k=k-1;
                        end
                    end
                    if prevL(v)=='?'
                        prevL(v)=dirLetter(i);
                        prev(v)=u;
                    end
                    if any(RobotAddr==v)%If the robot is the node we explore                
                       bestroboloc=v;
                        RobotFound=1;%Robot flag gets triggered and we break
                        break;
                    end
                end
            end
        end
        
    end
    source=nextsource;
    dist=dist+1;
end
goalf=bestroboloc;
% path = zeros(1,dist);
% pathL = repmat(' ',1,dist);
% path(dist) = prev(v);
% pathL(dist) = prevL(v);
% for l = dist-1:-1:1
%     path(l) = prev(path(l+1));
%     pathL(l) =prevL(path(l+1));
% end
% PathL=fliplr(PathL);
end