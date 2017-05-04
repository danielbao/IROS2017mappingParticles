function [movecount,k,nodecount] = ClosestFrontier(k,itr)
% ClosestFrontier is a demonstration of mapping of a completely connected
% and bounded 2D discrete grid space with k particles that move uniformly.
% The permissible moves are left, right, up and down. Each move is one pixel
% in length. All particles move in the same direction unless stopped by
% black obstacles.
% The algorithm for motion planning moves the particles in red to frontier
% cells in blue until no frontier cells remain.
% Inputs: k- number of particles, itr= number of iterations
% outputs: movecount- number of moves taken for mapping, k= number of
% partilcles and nodecount- vector with number of frontier cells in each
% move.
% The DIJKSTRA distance is calculated in each cycle to find the shortest
% distance from all particles to the frontiers.
% There are 21 maps to choose from- 1 through 26. Map specifications can be
% found in BlockMaps

%  Aaron T. Becker
%     atbecker@uh.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global G;
if nargin<1%if no inputs are provided
    k = 200;%default num particles
    itr=1;
end
G.fig = figure(1);
set(gcf,'Renderer','OpenGL');%setting graph renderer to use OpenGL
G.mapnum =24;%22; identifier for map, 0-26; look at blockMaps to identify each map
G.movecount = 0;%number of moves made
G.movetyp = [-1,0;0,1;1,0;0,-1];%array for making moves; begins up, right, down, left down each row
movecount=G.movecount;
G.drawflag=1; % Default 1, draw G.fig on. Set 0 for draw G.fig off.
G.videoflag=0;%default 0, set to 1 if video is to be made
clc
%% Making a video demonstration. makemymovie gets the current frame of imge and adds to video file
format compact
MOVIE_NAME =['Closest Frontier_map',num2str(G.mapnum),'_',num2str(k),'robots','_video8']; %Change video name here
writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
set(writerObj,'Quality',100);
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
[G.obstacle_pos,G.free,G.robvec,G.Moves] = SetupWorld(G.mapnum);%setup global arrays for master obstacle positions etc.
set(G.fig ,'KeyPressFcn',@keyhandler,'Name','Massive Control','color','w')
G.maxX = size(G.obstacle_pos,2);%x-dimension of obstacles in number of columns
G.maxY = size(G.obstacle_pos,1);%y-dimension of obstacles in number of rows
G.colormap = [ 1,1,1; %Empty = white  0
    0.5,0.5,0.5; %undiscovered= grey 1
    1,1,1; %robot= white square with red circle 2
    0,0,0;%obstacle= black           3
    0,0,1;%boundary cells/frontier= blue      4
    ];

randRobots=randperm(numel(G.robvec)); %randRobots: 1:num_empty_spaces, randomly arranged; randomize robots in robot positions
G.robvec(randRobots(k+1:end))=0; % locations of the k robots

RobotVisits=zeros(size(G.obstacle_pos)); %Set blind map to zeros. We want to build path in this map
map_expected=zeros(size(G.obstacle_pos)); %Set zeros initially. We update the expected location if each particle in this map
mapped_obstacles=zeros(size(G.obstacle_pos)); %Map is updated when obstacles are found
frontier_exp= zeros(size(G.obstacle_pos)); %Map to update the locations of frontiers
updateMap() %Update the map with the information from all the seperate maps
set(gca,'box','off','xTick',[],'ytick',[],'ydir','normal','Visible','on');%create graph without all of the axes
axis equal
axis tight
updateTitle() %Update the values displayed in the title
hold on
makemymovie()
if G.drawflag==1
    drawcirc()
end
%End of initialization
CF() % Closest Frontier mapping algorithm
%% CF repeatedly moves particles to frontier cells until there are no more frontier cells left
    function CF()
        iter=1;
        while(nnz(frontier_exp)>0)%While there are still unknowns in our frontier map, DFS beigin
            frontier_vec=G.boundvec; %refresh local variable to global current locations of frontiers
            roboloc=G.roboloc; %refresh local locations to global current locations of particles
            moveSeq = DijkstraForBoundary_mod(G.update_map,roboloc,frontier_vec); %the shortest path to a frontier cell selected by expanding from particles
            steps = min(inf,numel(moveSeq));%get the minimum number of steps from the Dijkstra's algo
            for mvIn =1:steps%move to the frontier
                moveto(moveSeq(mvIn)); %Implement moves on all particles
                nodecount(iter)=nnz(frontier_exp);%update the nodes visited in each step
                iter=iter+1;%increment the iterations taken, not moveCount, used for efficiency
                makemymovie()
            end %end DFS
        end
        for i=1:5
            makemymovie()
        end
    end
%% nodes updates the frontier_exp which is the matrix of frontiers explored
    function nodes(robIndex)
        %For each robot, check if the cell in direction mv is unknown
        for mv_type=1:4%check in 4 directions
            for c = 1:numel(robIndex);%check for every robot
                i2 = G.ri(robIndex(c))+G.movetyp(mv_type,2);%change x-indice
                j2 = G.ci(robIndex(c))+G.movetyp(mv_type,1);%change y-indice
                % if the cell has never been visited and isn't an obstacle
                if RobotVisits(i2,j2) == 0 && mapped_obstacles(i2,j2) == 0
                    frontier_exp(i2,j2)=1;%It's a frontier!
                else
                    frontier_exp(i2,j2)=0;%It isn't a frontier!
                end
            end
        end
    end


%% Apply move called by moveto to all the robots
    function rvec2 = applyMove(mv, rvecIn)%mv=move selected; rvecIn=robots/particles locations
        rvec2 = zeros(size(rvecIn));
        if mv==1 || mv==4 %collision check for left and down
            for ni = 1:numel(rvecIn)%G.Moves(ni,mv) returns 1 if there is a robot after the movement mv
                if rvecIn(G.Moves(ni,mv)) ~= 1%If a robot isn't there at left or down
                    rvec2(G.Moves(ni,mv)) = rvecIn(ni);%We move to the new address and return it
                    rvecIn(ni)=0;%clear rvecIn for robot ni
                else
                    rvec2(ni)=rvecIn(ni);%return old address for robot
                end
                rvecIn(ni)=rvec2(ni);%robot stays there
            end
        else %collision check for up and right
            for ni=numel(rvecIn):-1:1%for the rest of the robots
                if rvecIn(G.Moves(ni,mv)) == 0%equivalent to ~=0, if there isn't a robot to the right or above
                    rvec2(G.Moves(ni,mv)) = rvecIn(ni);
                    rvecIn(ni)=0;%clear input for robot ni
                else
                    rvec2(ni)=rvecIn(ni);%
                end
            end
        end
    end

%% takes input from user
    function keyhandler(src,evnt) %#ok<INUSL>
        if strcmp(evnt.Key,'s')
            imwrite(flipud(get(G.axis,'CData')+1), G.colormap, '../../pictures/png/MatrixPermutePic.png');
        else
            moveto(evnt.Key)
        end
    end
%% moveto(key) calls apply move and updates the map
    function moveto(key)
        G.movecount = G.movecount+1;%increment movecount everytime a move is applied
        mv=0;%reset move everytime
        if strcmp(key,'leftarrow') || strcmp(key,'l')|| strcmp(key,'1')  %-x
            mv = 1;
        elseif strcmp(key,'rightarrow')|| strcmp(key,'r')|| strcmp(key,'2')  %+x
            mv = 2;
        elseif strcmp(key,'uparrow')|| strcmp(key,'u')|| strcmp(key,'3')  %+y
            mv = 3;
        elseif strcmp(key,'downarrow')|| strcmp(key,'d') || strcmp(key,'4') %-y
            mv = 4;
        end
        if mv>0%do the move
            map_expected=G.im2;
            
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
            G.robvec = applyMove(mv, G.robvec);%move robots
            updateMap()
            updateTitle()
            if G.drawflag==1
                drawcirc()%draw each robot again
            end
            drawnow
            makemymovie()
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
        current_map = ones(size(G.obstacle_pos)); % 1== undiscovered
        current_map(G.free) = 2*G.robvec; %2 = particles on map
        
        G.im2=zeros(size(current_map)); %Set particles matrix to zero. This matrix will have 1 where particles are and zero otherwise
        G.im2(G.free(G.robvec>0))=1;%Set particles matrix to 1 where the robots are
        RobotVisits=G.im2+RobotVisits; %RoboVisits stores cells already visited by particles
        robIndex = find(G.robvec);%get positions of robots again
        nodes(robIndex);
        current_map(RobotVisits==0)=1; % 1= undiscovered, set it to undiscovered if there no robot has visited there
        frontier_exp(current_map==2)=0;%current_map is 2 when it has been visited; therefore it isn't a frontier
        map_expected(current_map~=1)=0;%if current map is not visited, then there must be nothing expected there
        mapped_obstacles = mapped_obstacles | map_expected;%OR the expected and mapped obstacles to update obstacles
        frontier_exp(mapped_obstacles)=0;%all of our obstacles are refreshed to not be frontiers
        current_map(frontier_exp==1)=4; % 4 = frontier cells
        current_map(mapped_obstacles==1)=3; % 3 = obstacles
        G.update_map=zeros(size(current_map));%reset update_map
        G.update_map(current_map==3)=1;%reset our unknowns and obstacles to be undiscovered
        G.update_map(current_map==1)=1;%reset undiscovered to be undiscovered
        G.boundvec=find(current_map== 4);%set boundaries to be frontiers
        G.roboloc=find(current_map== 2);%set robot locations to where they have visited
        [G.robscatx,G.robscaty]=find(current_map== 2);%store scatter plot locations of the robot
        colormap(G.colormap(unique(current_map)+1,:));
        if G.drawflag==1
            G.axis=imagesc(current_map); %show map that updates as robots explore
        end
    end
%% Update title when called
    function updateTitle()
        if nnz(frontier_exp)==1%Grammatical condition if there is only 1 frontier cell
            FC=' Frontier Cell';
        else
            FC=' Frontier Cells';
        end
        title([num2str(G.movecount), ' moves, ',num2str(sum(G.robvec)),' particles, ', num2str(nnz( frontier_exp)), FC,',Iteration :', num2str(itr)])
    end
%% setup map
    function [blk,free,robvec,Moves] = SetupWorld(mapnum)
        % blk is the position of obstacles
        % free is the index of the free spaces in blk
        % robvec is a binary vector where the ith element is true if there
        % is a robot at free(i).
        blk = blockMaps(mapnum); % function returns the binary map specified by mapnum
        free = find(blk==0); % free spaces are where the obstacles are 0
        robvec = ones(size(free)); % put a robot in every free space
        [ri,ci] = find(blk==0);
        G.ri=ri;
        G.ci=ci;
        Moves = repmat( (1:numel(free))',1,4); %repeat free four times across
        world = -blk;
        world(free) = 1:numel(free);%assign a vector to world at location of free to
        %represent each possible move at free space
        for i = 1:numel(free)%iterate through free
            r = ri(i);
            c = ci(i);
            if blk(r,c-1) == 0%using beginning LR UD conventions, if you can move left
                Moves(i,1) = world(r,c-1);%store possible left move in Moves from world's left cell
            end
            if blk(r,c+1) == 0%move right
                Moves(i,2) = world(r,c+1);%store possible right move in Moves from world's right cell
            end
            if blk(r+1,c) == 0%move down
                Moves(i,3) = world(r+1,c);%store possible down move in Moves from world's down cell
            end
            if blk(r-1,c) == 0%move up
                Moves(i,4) = world(r-1,c);%store posible up move in Moves from world's down cell
            end
        end
        
    end
end