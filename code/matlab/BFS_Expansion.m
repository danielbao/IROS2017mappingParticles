function [pathL,goalf] = BFS_Expansion( Graph, source, goal, value )
% BFS_Expansion BFS of free spaces searching for frontier spaces along
% levels to evaluate them based off of a valueMap generated from the
% indices of the boundaries and 
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
mincost=100000000000;
level=0;
lowvalflag=0;%Flag for when a minimum is found
expandedFrontiers=[];
% firstFrontier=0;%Temp storage for the greedy first v; not used right now
% Source node will be selected first from the indices

while numel(Q)>0&& lowvalflag==0%While there are free spaces to explore
    %And there has been no minimum visited
    %We make our direction matrix
    dirs = [ 0,-1;% left
        -1,0; % up
        0,1;  % right
        1,0;];  % down
    [~,ind] = min(dist(Q)); % Source nodes will be selected first
    u = Q(ind);%We select a u to expand and take its
    %index
    j=find(source==ind);
    source(j)=[];% remove u from source
    Q(ind) = []; % remove u from Q
    dirLetter = ['l','u','r','d'];
    [ui,uj] = ind2sub(size(Graph),u); % Get row column index of u
    frontierFound=0;
    frontiers=zeros(10000); %Pre-allocated array of frontiers and t
    %values for address evaluation
    while frontierFound==0%We stop a source expansion once we hit a frontier
        for i = 1:size(dirs,1)%For each direction
            if (dirs(i,1)+ui>0)&&(dirs(i,1)+ui<size(Graph,1))
                if (dirs(i,2)+uj>0&&(dirs(i,2)+uj<size(Graph,2)))
                    v = sub2ind(size(Graph), dirs(i,1) + ui,  dirs(i,2) +uj);
                    %We must check the bounds of our expansion

                end
                
            end
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
                            frontierFound=1;%frontierFound allows us to get
                            %                            out of the while loop
                            lowvalflag=1;%lowvalflage allows us to skip
                            %frontier evaluation and go to
                            break;
                        end
                        frontiers(v)=value(v);%store the value of the frontier
                        %in its respective address.
                        frontierFound=1;%Get out of the while loop once we
                        % are finished with this level's expansion in all
                        % directions
                    end
                end
            end
            
        end
    end
    if numel(source)==0%If there are no more source cells to expand from
        break;%Get out of the while loop and calculate values;
    end
end
%Now we must calculate the levels we need to expand and expand to those
%levels
Frontierslvl=zeros(2,100000);% Array to store the new frontiers found by 
% expanding ith level
indFrontiers=find(frontiers);%Get the indices of the frontiers
if lowvalflag~=1
    levels=getLevels(frontiers);
    %We start the first expansion from our previous frontiers
    for i=1:levels%expand each level;
        dirs = [ 0,-1;% left
        -1,0; % up
        0,1;  % right
        1,0;];  % down
        %We take a direction and we apply it to a source node from
        %frontiers
        % Select a source node
        while(size(indFrontiers)>0)
            u=indFrontiers(1);
            indFrontiers(1)=[];%remove source frontier from the frontiers
            [ui,uj] = ind2sub(size(Graph),u);%Get subscripted indices of frontier
            
            for j = 1:size(dirs,1)%For each direction
                if (dirs(i,1)+ui>0)&&(dirs(i,1)+ui<size(Graph,1))
                    if (dirs(i,2)+uj>0&&(dirs(i,2)+uj<size(Graph,2)))
                        v = sub2ind(size(Graph), dirs(i,1) + ui,  dirs(i,2) +uj);
                        %We must check the bounds of our expansion
                        
                    end
                    
                end
                %Get the expanded node
                if any(goal==v)&&Graph(v)==0%If it's a frontier and it's a free
                    %space
                    Frontierslvl(i,v)=value(v);%Store this frontier in the array
                    %to expand for the next level.
                    if level+value(v)<=-1%If we find a better frontier with lower
                        %value
                        lowvalflag=1;%Trigger low value flag again and
                        indFrontiers=[];%Get out of the indFrontier While loop
                        break;
                    end
                    
                end
                
            end
        end

        if(lowvalflag==1)%endpoint for the minimum found
            break;
        end
    end

end
%Now we calculate the path required to get to the frontier
goalf=v;%We need to make sure v is the right "last" frontier we located
path = zeros(1,dist(v));
pathL = repmat(' ',1,dist(v));
path(dist(v)) = prev(v);
pathL(dist(v)) = prevL(v);
for k = dist(v)-1:-1:1
    path(k) = prev(path(k+1));
    pathL(k) =prevL(path(k+1));
end
goalbackmat=zeros(size(goalmat));
goalbackmat(goalf)=1;
goalbackmat=flipud(goalbackmat);
goalf=find(goalbackmat); %finding location of goal frontier cell to be returned.
end

function levels=getLevels(frontiers)
%%getLevels returns the amount of levels needed to expand the
indFrontiers=find(frontiers);%Get the indices of the frontiers
minFrontiers=min(frontiers);%Get minimum value of the frontiers
levels=0;%Default levels to 0 because there may be no 
for i=1:size(indFrontiers)
    if minFrontiers>=0%No Frontier has a value
        %This means that we can expand twice because there may be a frontier
        %cell with a value of -3 in 2 levels which justifies the expansion.
        levels=2;
        break;
    end
    if indFrontiers(i)<=-1%If there is a frontier cell with a value of -1
        %We can expand the level by 2 to see if there is a frontier cell 
        levels=1;
    
    end
    
end
%We then wouldn't have to expand because there would be no

end

