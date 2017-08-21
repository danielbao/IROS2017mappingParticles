function [movecount,k,nodecount,init_config] = ClosestFrontier_VAF(k,itr,max_steps,config_flag,starting_config)
% ClosestFrontier is a demonstration of mapping a completely connected and 
% bounded 2D discrete grid space with k particles that move uniformly.
% The permissible moves are left, right, up and down. Each move is one pixel
% in length. All particles move in the same direction unless stopped by
% black obstacles.
% This algorithm for motion planning moves the particles in red to frontier
% cells in blue until no frontier cells remain.
% Inputs: k= number of particles, itr= number of iterations
% outputs: movecount= number of moves taken for mapping, k= number of
% particles and nodecount= vector with number of frontier cells in each
% move.
% The DIJKSTRA distance is calculated in each cycle to find the shortest
% distance from all particles to the frontiers.
% There are 33 maps to choose from: 1 through 22 being
% created through matrices and 24 through 33 being image read maps.
% Map specifications can be found in blockMaps.m and selected by G.mapnum
% 
%
%
%
%
%  Authors:
%  Aaron T. Becker
%     atbecker@uh.edu
%  Arun Mahadev
%     avm.rensol@gmail.com
%  Edited by: Daniel Bao
%     dzbao@uh.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Begin Initialization
global G; %Global variable used to store all of our matrices
if nargin<1 %If no inputs are provided
    k = 1000;%Default num particles
    itr=1;
    config_flag=0; % Whether or not to use the previous configuration
    max_steps=250; % Maximum number of steps we want to take
    starting_config=0; % Inputted configuration
end
G.fig = figure(1);
set(gcf,'Renderer','OpenGL');%use OpenGL for graphs, not sure if other
%settings may produce better results
G.mapnum = 28;% Identifier for map, 0-26; look at blockMaps to identify each map
G.movecount = 0;%Number of moves made
G.movetyp = [-1,0;0,1;1,0;0,-1];%Array for making moves;
                                %Each row is up, right, left, down
movecount=G.movecount;
G.drawflag=1; % Default 1, draw G.fig on. Set 0 for draw G.fig off.
G.videoflag=0;% Default 0, set to 1 if video is to be made
G.playflag=0;%flag for user playing with keyboard inputs
G.valueflag=0; %flag for user inputting values
G.value=0; %variable for storing the value wanted to search for.
% The keys for values
% 0 is free space
% 1 is unknown
% 2 is robotvisited
% 3 is obstacle
% 4 is frontier
if(G.valueflag==1)
   prompt='What neighbor cell do you want to prioritize frontier exploration?';
   prompt=[prompt newline '0 for free space' newline];
   prompt=[prompt '1 from unknown cells' newline];
   prompt=[prompt '2 for robotvisited' newline];
   prompt=[prompt '3 for obstacle'];
   prompt=[prompt newline '4 for frontier(no effect?)'];
   temp_input=input(prompt)
   G.value=temp_input;
end
clc
%% Making a video demonstration. makemymovie gets the current frame of imge and adds to video file
format compact
MOVIE_NAME =['Closest Frontier_map',num2str(G.mapnum),'_',num2str(k),'robots','_video8']; %Change video name here
writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
set(writerObj,'Quality',100);%Default settings for video
writerObj.FrameRate=30;
open(writerObj);
    function makemymovie()% Call after each frame is generated
        if G.videoflag==1
            figure(G.fig)
            F = getframe(G.fig);
            writeVideo(writerObj,F.cdata);
        end
    end
%% Setup map, matrices and initiate mapping
[G.obstacle_pos,G.free,G.robvec,G.Moves] = SetupWorld(G.mapnum);
set(G.fig ,'KeyPressFcn',@keyhandler,'Name','Massive Control','color','w')
G.maxX = size(G.obstacle_pos,2);%x-dimension of obstacles in number of columns
G.maxY = size(G.obstacle_pos,1);%y-dimension of obstacles in number of rows
G.colormap = [ 1,1,1; %Empty=white  0
    0.5,0.5,0.5; %undiscovered=grey 1
    1,1,1; %robot=white square with red circle 2
    0,0,0;%obstacle=black 3
    0,0,1;%boundary cells/frontier=blue 4
    ];

if config_flag==1
    G.robvec=starting_config;
else
    randRobots=randperm(numel(G.robvec)); %randomize robots in their positions
    G.robvec(randRobots(k+1:end))=0; % locations of the k robots
end

init_config=G.robvec; % We store the first locations (linear indices) of the robots
% after they're randomized
RobotVisits=zeros(size(G.obstacle_pos)); %Set blind map to zeros. We want to build path in this map
map_expected=zeros(size(G.obstacle_pos)); %Set zeros initially. We update the expected location of each particle in this map
mapped_obstacles=zeros(size(G.obstacle_pos)); %Map is updated when obstacles are found
frontier_exp=zeros(size(G.obstacle_pos)); %Map to update the locations of frontiers
valueMap=zeros(size(G.obstacle_pos)); %Map to update with the values of frontier cells
                                      % for the weighted exploration and
                                      % mapping
updateMap() %Update the map with the information from all the seperate maps
set(gca,'box','off','xTick',[],'ytick',[],'ydir','normal','Visible','on');%Create graph without all of the axes
axis equal
axis tight
updateTitle() %Update the values displayed in the title
hold on

if G.drawflag==1
    drawcirc()
end
for m=1:60%Beginning frames drawn
    makemymovie();
end
%End of initialization
if(G.playflag==0)
    CF() %Closest Frontier mapping call
end

for m=1:60 %Ending frames result drawn
    makemymovie();
end
%% CF repeatedly moves particles to frontier cells until there are no more frontier cells left
    function CF()
        iter=1;
        while(nnz(frontier_exp)>0&&G.movecount<max_steps)%While there are still unknowns, DFS begins
            frontier_vec=G.boundvec; %Refresh local variable to global current locations of frontiers
            roboloc=G.roboloc; %Refresh local locations to global current locations of robots
            moveSeq = BFS_Expansion_AVM(G.update_map,roboloc,frontier_vec, valueMap); 
            %DijkstraForBoundary_mod
            %BFS_Expansion
            %The shortest path to a frontier cell is selected by expanding from particles
            %Now this uses the mod_star to select a path to the lowest cost
            %frontier cell; not necessarily the closestOne
            steps = max(0,numel(moveSeq));%Get the minimum number of steps from Dijkstra's
            for mvIn =1:steps%Move to the frontier on all particles
                moveto(moveSeq(mvIn));
                nodecount(iter)=nnz(frontier_exp);%Update the nodes visited in each step
                iter=iter+1;
                makemymovie()
            end %end DFS
        end
        for i=1:5
            makemymovie()
        end
    end
%% nodes updates frontier_exp
    function nodes(robIndex)
        %For each robot, check if the cell in direction mv is unknown
        for mv_type=1:4%Check in 4 directions
            for c = 1:numel(robIndex)%Check for every robot
                i2 = G.ri(robIndex(c))+G.movetyp(mv_type,2);%Change x-index
                j2 = G.ci(robIndex(c))+G.movetyp(mv_type,1);%Change y-index
                %If the cell has never been visited and isn't an obstacle
                if RobotVisits(i2,j2) == 0 && mapped_obstacles(i2,j2) == 0
                    frontier_exp(i2,j2)=1;%It's a frontier!
                else
                    frontier_exp(i2,j2)=0;%It isn't a frontier!
                end
            end
        end
    end


%% Apply move called by moveto to all the robots
    function rvec2 = applyMove(mv, rvecIn)%mv=move selected; rvecIn=robots locations
        rvec2 = zeros(size(rvecIn));%Result matrix to store moved robots
        if mv==1 || mv==4 %Collision check for left and up
            for ni = 1:numel(rvecIn)%G.Moves(ni,mv) returns 1 if there is a robot after the movement mv
                if rvecIn(G.Moves(ni,mv)) ~= 1%If a robot isn't there at left or down
                    rvec2(G.Moves(ni,mv)) = rvecIn(ni);%Set result to new location
                    rvecIn(ni)=0;%Clear rvecIn for robot
                else
                    rvec2(ni)=rvecIn(ni);%Update result with old location
                end
                rvecIn(ni)=rvec2(ni);%Set old locations with new locations
            end
        else %Collision check for down and right
            for ni=numel(rvecIn):-1:1%For the rest of the robots
                if rvecIn(G.Moves(ni,mv)) == 0%Equivalent to ~=1
                    rvec2(G.Moves(ni,mv)) = rvecIn(ni);%Set result to new location
                    rvecIn(ni)=0;%clear input for robot ni
                else
                    rvec2(ni)=rvecIn(ni);%Update result with old location
                end
                rvecIn(ni)=rvec2(ni);%Set old locations with new location
            end
        end
    end

%% keyhandler takes input from user
    function keyhandler(src,evnt) %#ok<INUSL>
        if strcmp(evnt.Key,'s')%s is the cancel key
            imwrite(flipud(get(G.axis,'CData')+1), G.colormap, '../../pictures/png/MatrixPermutePic.png');
        else
            moveto(evnt.Key)
        end
    end
%% moveto(key) calls apply move and updates the map
    function moveto(key)
        G.movecount = G.movecount+1;%Increment movecount everytime a move is applied
        mv=0;%Reset move selector
        if strcmp(key,'leftarrow') || strcmp(key,'l')|| strcmp(key,'1')  %-x
            mv = 1;
        elseif strcmp(key,'rightarrow')|| strcmp(key,'r')|| strcmp(key,'2')  %+x
            mv = 2;
        elseif strcmp(key,'uparrow')|| strcmp(key,'u')|| strcmp(key,'3')  %+y
            mv = 3;
        elseif strcmp(key,'downarrow')|| strcmp(key,'d') || strcmp(key,'4') %-y
            mv = 4;
        end
        if mv>0%Do the move
            map_expected=G.im2;%im2 is the local particle location matrix
            
            % map_expected has 1 where robot is expected to be
            if mv==1
                map_expected = circshift(map_expected,[0 -1]);%shift map left by 1
            elseif mv==2
                map_expected = circshift(map_expected,[0 1]);%shift map right by 1
            elseif mv==3
                map_expected = circshift(map_expected,[1 0]);%shift map down by 1
            else
                map_expected = circshift(map_expected,[-1 0]);%shift map up by 1
            end
            %G.movecount = G.movecount+1;commented out because it is a
            %double movecount
            G.robvec = applyMove(mv, G.robvec);%Move robots and put it in actual positions
            updateMap()
            updateTitle()
            if G.drawflag==1
                drawcirc()%Draw each robot again
            end
            drawnow
            if G.videoflag==1
                makemymovie()
            end
        end
    end
%% Drawing scatter circles in the location of particles
    function drawcirc()
        h=scatter(G.robscaty,G.robscatx,'r','filled');%create a new variable with the scatterplot
        currentunits = get(gca,'Units');%use current Units
        set(gca, 'Units', 'Points');%graph the current Units and Points
        axpos = get(gca,'Position');
        set(gca, 'Units', currentunits);
        markerWidth = .8/diff(xlim)*axpos(3); % Calculate Marker width in points
        set(h,'SizeData', markerWidth^2)%graph with data and squared width
        drawnow
    end
%% Updating the map
    function updateMap()
        current_map = ones(size(G.obstacle_pos)); %1=undiscovered
        current_map(G.free) = 2*G.robvec; %2=particles on map
        
        G.im2=zeros(size(current_map)); %Set particles matrix to zero. This matrix will have 1 where particles are and zero otherwise
        G.im2(G.free(G.robvec>0))=1;%Set particles matrix to 1 where the robots are
        RobotVisits=G.im2+RobotVisits; %RoboVisits stores cells already visited by particles
        robIndex = find(G.robvec);%Get positions of robots again
        nodes(robIndex);
        current_map(RobotVisits==0)=1; %1=undiscovered, set it to undiscovered if there no robot has visited there
        frontier_exp(current_map==2)=0;%Current_map is 2 when it has been visited by a robot;
                                        %therefore it isn't a frontier
        map_expected(current_map~=1)=0;%If current map is a robot, then there must be free
        mapped_obstacles = mapped_obstacles | map_expected;%OR the expected and mapped obstacles to update obstacles
        frontier_exp(mapped_obstacles)=0;%All of our obstacles are refreshed to not be frontiers
        current_map(frontier_exp==1)=4; %4=frontier cells
        current_map(mapped_obstacles==1)=3; %3=obstacles
        temp_map=current_map;
        temp_map(G.free)=0;%0 equals free space;
        G.update_map=zeros(size(current_map));%reset update_map
        G.update_map(current_map==3)=1;%Reset our unknowns and obstacles to be unoccupied
        G.update_map(current_map==1)=1;%Reset undiscovered to be unoccupied
        G.boundvec=find(current_map==4);%Set boundaries to be frontiers
        G.roboloc=find(current_map==2);%Set robot locations to where they have visited
        [G.robscatx,G.robscaty]=find(current_map== 2);%Store scatter plot locations of the robot
        colormap(G.colormap(unique(current_map)+1,:));
        if G.drawflag==1
            G.axis=imagesc(current_map); %Show map that updates as robots explore
        end
        % Strategy for the Value Map
        % Loop through current_map; If there is a 3 (obstacle) next to a
        % 4(frontier), we give it a value of -(how many of frontier cells
        % around it)
        % To get values for other types of cells, we only need to change
        % it some other number for value
        % We need to create a map with all of the free spaces
        valueMap = checkNeighbors(temp_map, G.value);
%         disp(flipud(valueMap))
%         drawcirc();
%         disp(newline)
%       Code for checking checkNeighbors. It works now
    end
%% checkNeighbors checks the neighbor of each cell for a certain value
    function Neighbors=checkNeighbors(Array, value)
        Neighbors=zeros(size(Array));
        [m,n]=size(Array);
        for i=1:m
            for j=1:n
                if(Array(i,j)==4)% If it's a frontier
                    if(i-1~=0)% If the upper neighbor isn't out of bounds
                        if(Array(i-1,j)==value)% If that cell is a valued cell
                            Neighbors(i,j)=Neighbors(i,j)-1;% Lower its value
                        end
                    end
                    if(i+1<=m)% If the lower neighbor isn't out of bounds
                        if(Array(i+1,j)==value)% If that cell is a valued cell
                            Neighbors(i,j)=Neighbors(i,j)-1;
                        end
                    end
                    if(j-1~=0)% If the left neighbor isn't out of bounds
                        if(Array(i,j-1)==value)% If that cell is a valued cell
                            Neighbors(i,j)=Neighbors(i,j)-1;
                        end
                    end
                    if(j+1<=n)% If the right neighbor isn't out of bounds
                        if(Array(i,j+1)==value)% If that cell is a valued cell
                            Neighbors(i,j)=Neighbors(i,j)-1;
                        end
                    end
                end
            end
        end
        
    end
%% updateTitle updates title when called
    function updateTitle()
        if nnz(frontier_exp)==1%Grammatical condition if there is only 1 frontier cell
            FC=' frontier cell';
        else
            FC=' frontier cells';
        end
        title([num2str(G.movecount), ' moves, ',num2str(sum(G.robvec)),' particles, ', num2str(nnz( frontier_exp)), FC,', ', num2str(nnz(G.free)), ' free cells'])
    end
%% SetupWorld setups map
    function [blk,free,robvec,Moves] = SetupWorld(mapnum)
        %blk is the position of obstacles
        %free is the index of the free spaces in blk
        %robvec is a binary vector where the ith element is true if there
        %is a robot at free(i).
        blk = blockMaps(mapnum); %Function returns the binary map specified by mapnum
        free = find(blk==0); %Free spaces are where the obstacles are 0
        robvec = ones(size(free)); % Robots can be randomized in the free space
        [ri,ci] = find(blk==0);
        G.ri=ri;
        G.ci=ci;
        Moves = repmat( (1:numel(free))',1,4); %Repeat free four times across
                                               %Represents the move
                                               %possibilities
        world = -blk;
        world(free) = 1:numel(free);%Assign a vector to world at location of free to
        %Represent each possible move at free space
        for i = 1:numel(free)%Iterate through free
            r = ri(i);
            c = ci(i);
            if blk(r,c-1) == 0%Using beginning LR UD conventions, if you can move left
                Moves(i,1) = world(r,c-1);%Store possible left move in Moves from world's left cell
            end
            if blk(r,c+1) == 0%Move right
                Moves(i,2) = world(r,c+1);%Store possible right move in Moves from world's right cell
            end
            if blk(r+1,c) == 0%Move down
                Moves(i,3) = world(r+1,c);%Store possible down move in Moves from world's down cell
            end
            if blk(r-1,c) == 0%Move up
                Moves(i,4) = world(r-1,c);%Store posible up move in Moves from world's up cell
            end
        end
        
    end
end