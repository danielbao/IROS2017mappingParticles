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
Graph=flipud(Graph);
value=flipud(value);
prevL = repmat('?',size(Graph));
prev = -1*ones(size(Graph));
path = []; %#ok<NASGU>
pathL = [];
goalmat=zeros(size(Graph));
goalmat(goal)=1;% the goals are where it's 1
goalmat=flipud(goalmat);
goal=find(goalmat);% Get the indice after you flip the matrix
% while Q is not empty:
mincost=100000000000;
level=0;
lowvalflag=0;
dist=0;
PassSource=source;
% Source node will be selected first from the indices
while lowvalflag==0%While there are free spaces to explore
    %And there has been no minimum visited
    %We make our direction matrix
    dirs = [ 0,-1;% left
        -1,0; % up
        0,1;  % right
        1,0;];  % down
    
    k=0;
    %values for address evaluation
    while numel(source)>0%Logic might need help here
        u=source(1);
        source(1)=[];
        dirLetter = ['l','u','r','d'];
        [ui,uj] = ind2sub(size(Graph),u); % Get row column index of u
        for i = 1:size(dirs,1)%For each direction
            if dirs(i,1) + ui>=1 && dirs(i,2) +uj >=1 && dirs(i,1) +ui <= size(Graph,1) && dirs(i,2) +uj <= size(Graph,2)
                v = sub2ind(size(Graph), dirs(i,1) + ui,  dirs(i,2) +uj);
                %We get the cell in the direction
                if Graph(v) == 0 %only try to move if the vertex is 0
                    if isempty(find(source==v))
                        k=k+1;
                        nextsource(k)=v;
                        if numel(find(nextsource==v))>1
                            nextsource(k)=[];
                            k=k-1;
                        end
                    end
                    prevL(v)=dirLetter(i);
                    prev(v)=u;
                    
                    if any(goal==v)%If there's a frontier
                        
                        if value(v)<=-2 && level==0
                            lowvalflag=1;%lowvalflage allows us to skip
                            mincost=value(v);
                            bestfrontierloc=v;
                            %frontier evaluation and go to
                            break;
                        elseif value(v)< mincost
                            mincost=value(v);
                            if level==0
                                bestfrontierloc=v;
                            end
                        end
                    end
                end
            end
        end
    end
    if lowvalflag==0
        dist=dist+1;
        if level==0
            if mincost==0
                gracelvl=2;
            elseif mincost==-1
                gracelvl=1;
            end
        elseif level==1
            if gracelvl==1
                if mincost>=-1
                    v=bestfrontierloc;
                end
                dist=dist-1;
                lowvalflag=1;
            end
        elseif level==2
            if mincost>=-2
                v=bestfrontierloc;
                dist=dist-2;
            end
            lowvalflag=1;
        end
        if mincost<=0
            level=level+1;
        end
        
        
    end
end
[frontdist,goalr, prevL,prev]= pathcalc(Graph,PassSource,v);
% %Now we calculate the path required to get to the frontier
path = zeros(1,frontdist);
pathL = repmat(' ',1,frontdist);
path(frontdist) = prev(goalr);
pathL(frontdist) = prevL(goalr);
for l = frontdist-1:-1:1
    path(l) = prev(path(l+1));
    pathL(l) =prevL(path(l+1));
end
pathL=fliplr(pathL);
goalbackmat=zeros(size(goalmat));
goalbackmat(v)=1;
goalbackmat=flipud(goalbackmat);
goalf=find(goalbackmat); %finding location of goal frontier cell to be returned.
end

function [dist,goalf, prevL,prev]= pathcalc(Graph,RobotAddr,Frontier)
 Graph=flipud(Graph);
Robomat=zeros(size(Graph));
Robomat(RobotAddr)=1;
Robomat=flipud(Robomat);
RobotAddr=find(Robomat);
source=Frontier;
prevL = repmat('?',size(Graph));
prev = -1*ones(size(Graph));
path = []; %#ok<NASGU>
RobotFound=0;
dist=0;
dirs = [ 0,-1;% left
    -1,0; % up
    0,1;  % right
    1,0;];  % down
dirLetter = ['r','d','l','u'];
while RobotFound==0%Logic might need help here
    k=0;
    while numel(source)>0%Logic might need help here
        u=source(1);
        source(1)=[];
        [ui,uj] = ind2sub(size(Graph),u); % Get row column index of u
        for i = 1:size(dirs,1)%For each direction
            if dirs(i,1) + ui>=1 && dirs(i,2) +uj >=1 && dirs(i,1) +ui <= size(Graph,1) && dirs(i,2) +uj <= size(Graph,2)
                v = sub2ind(size(Graph), dirs(i,1) + ui,  dirs(i,2) +uj);
                %We get the cell in the direction
                if Graph(v) == 0 %only try to move if the vertex is 0
                    if isempty(find(source==v))
                        k=k+1;
                        nextsource(k)=v;
                        if numel(find(nextsource==v))>1
                            nextsource(k)=[];
                            k=k-1;
                        end
                    end
                    prevL(v)=dirLetter(i);
                    prev(v)=u;
                    
                    if any(RobotAddr==v)%If there's a frontier
                        
                        RobotFound=1;
                        
                        break;
                    end
                end
            end
        end
    end
    source=nextsource;
    dist=dist+1;
end
goalf=v;
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