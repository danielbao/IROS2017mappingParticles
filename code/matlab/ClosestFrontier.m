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
if nargin<1
    k = 200;%num particles
    itr=1;
end
G.fig = figure(1);
set(gcf,'Renderer','OpenGL');
G.mapnum =24;%22;
G.movecount = 0;
G.movetyp = [-1,0;0,1;1,0;0,-1];
movecount=G.movecount;
G.drawflag=1; % Default 1, draw G.fig on. Set 0 for draw G.fig off.
G.videoflag=0;
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
%% Setup map, matrices and initite mapping
[G.obstacle_pos,G.free,G.robvec,G.Moves] = SetupWorld(G.mapnum);
set(G.fig ,'KeyPressFcn',@keyhandler,'Name','Massive Control','color','w')
G.maxX = size(G.obstacle_pos,2);
G.maxY = size(G.obstacle_pos,1);
G.colormap = [ 1,1,1; %Empty = white  0
    0.5,0.5,0.5; %undiscovered= grey 1
    1,1,1; %robot= white square with red circle 2
    0,0,0;%obstacle= black           3
    0,0,1;%boundary cells= blue      4
    ];

randRobots=randperm(numel(G.robvec)); %randRobots: 1:num_empty_spaces, randomly arranged
G.robvec(randRobots(k+1:end))=0; % locations of the k robots

RobotVisits=zeros(size(G.obstacle_pos)); %Set blind map to zeros. We want to build path in this map
map_expected=zeros(size(G.obstacle_pos)); %Set zeros initially. We update the expected location if each particle in this map
mapped_obstacles=zeros(size(G.obstacle_pos)); %Map is updated when obstacles are found
frontier_exp= zeros(size(G.obstacle_pos)); %Map to update the locations of frontiers
updateMap() %Update the map with the information from all the seperate maps
set(gca,'box','off','xTick',[],'ytick',[],'ydir','normal','Visible','on');
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
        while(nnz(frontier_exp)>0)
            frontier_vec=G.boundvec; %current locations of frontiers
            roboloc=G.roboloc; %current locations of particles
            moveSeq = DijkstraForBoundary_mod(G.update_map,roboloc,frontier_vec); %the shortest path to a frontier cell selected by expanding from particles
            steps = min(inf,numel(moveSeq));
            for mvIn =1:steps
                moveto(moveSeq(mvIn)); %Implement moves on all particles
                nodecount(iter)=nnz(frontier_exp);
                iter=iter+1;
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
        for mv_type=1:4
            for c = 1:numel(robIndex);
                i2 = G.ri(robIndex(c))+G.movetyp(mv_type,2);
                j2 = G.ci(robIndex(c))+G.movetyp(mv_type,1);
                % if the cell has neer been visited and isn't an obstacle
                if RobotVisits(i2,j2) == 0 && mapped_obstacles(i2,j2) == 0
                    frontier_exp(i2,j2)=1;
                else
                    frontier_exp(i2,j2)=0;
                end
            end
        end
    end


%% Apply move called by moveto to all the robots
    function rvec2 = applyMove(mv, rvecIn)
        rvec2 = zeros(size(rvecIn));
        if mv==1 || mv==4 %colission check for left and down
            for ni = 1:numel(rvecIn)
                if rvecIn(G.Moves(ni,mv)) ~= 1
                    rvec2(G.Moves(ni,mv)) = rvecIn(ni);
                    rvecIn(ni)=0;
                else
                    rvec2(ni)=rvecIn(ni);
                end
                rvecIn(ni)=rvec2(ni);
            end
        else %collision check for up and right
            for ni=numel(rvecIn):-1:1
                if rvecIn(G.Moves(ni,mv)) == 0
                    rvec2(G.Moves(ni,mv)) = rvecIn(ni);
                    rvecIn(ni)=0;
                else
                    rvec2(ni)=rvecIn(ni);
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
        G.movecount = G.movecount+1;
        mv=0;
        if strcmp(key,'leftarrow') || strcmp(key,'l')|| strcmp(key,'1')  %-x
            mv = 1;
        elseif strcmp(key,'rightarrow')|| strcmp(key,'r')|| strcmp(key,'2')  %+x
            mv = 2;
        elseif strcmp(key,'uparrow')|| strcmp(key,'u')|| strcmp(key,'3')  %+y
            mv = 3;
        elseif strcmp(key,'downarrow')|| strcmp(key,'d') || strcmp(key,'4') %-y
            mv = 4;
        end
        if mv>0
            map_expected=G.im2;
            
            % map_expected has 1 where robot is expected to be
            if mv==1
                map_expected = circshift(map_expected,[0 -1]);
            elseif mv==2
                map_expected = circshift(map_expected,[0 1]);
            elseif mv==3
                map_expected = circshift(map_expected,[1 0]);
            else
                map_expected = circshift(map_expected,[-1 0]);
            end
            G.movecount = G.movecount+1;
            G.robvec = applyMove(mv, G.robvec);
            updateMap()
            updateTitle()
            if G.drawflag==1
                drawcirc()
            end
            drawnow
            makemymovie()
        end
    end
%% Drawing scatter circles in the location of particles
    function drawcirc()
        h=scatter(G.robscaty,G.robscatx,'r','filled');
        currentunits = get(gca,'Units');
        set(gca, 'Units', 'Points');
        axpos = get(gca,'Position');
        set(gca, 'Units', currentunits);
        markerWidth = .8/diff(xlim)*axpos(3); % Calculate Marker width in points
        set(h,'SizeData', markerWidth^2)
        drawnow
    end
%% Updating the map
    function updateMap()
        current_map = ones(size(G.obstacle_pos)); % 1== undiscovered
        current_map(G.free) = 2*G.robvec; %2 = particles on map
        
        G.im2=zeros(size(current_map)); %Set particles matrix to zero. This matrix will have 1 where particles are and zero otherwise
        G.im2(G.free(G.robvec>0))=1;
        RobotVisits=G.im2+RobotVisits; %RoboVisits stores cells already visited by particles
        robIndex = find(G.robvec);
        nodes(robIndex);
        current_map(RobotVisits==0)=1; % 1= undiscovered
        frontier_exp(current_map==2)=0;
        map_expected(current_map~=1)=0;
        mapped_obstacles = mapped_obstacles | map_expected;
        frontier_exp(mapped_obstacles)=0;
        current_map(frontier_exp==1)=4; % 4 = frontier cells
        current_map(mapped_obstacles==1)=3; % 3 = obstacles
        G.update_map=zeros(size(current_map));
        G.update_map(current_map==3)=1;
        G.update_map(current_map==1)=1;
        G.boundvec=find(current_map== 4);
        G.roboloc =find(current_map== 2);
        [G.robscatx,G.robscaty]=find(current_map== 2);
        colormap(G.colormap(unique(current_map)+1,:));
        if G.drawflag==1
            G.axis=imagesc(current_map); %show map that updates as robots explore
        end
    end
%% Update title when called
    function updateTitle()
        if nnz( frontier_exp)==1
            FC=' Frontier Cell';
        else
            FC=' Frontier Cells';
        end
        title([num2str(G.movecount), ' moves, ',num2str(sum(G.robvec)),' particles, ', num2str(nnz( frontier_exp)), FC,',Iteration :', num2str(itr)])
    end
%% setup map
    function [blk,free,robvec,Moves] = SetupWorld(mapnum)
        %  blk is the position of obstacles
        % free is the index of the free spaces in blk
        % robvec is a binary vector where the ith element is true if there
        % is a robot at free(i).
        blk = blockMaps(mapnum); % function returns the binary map specified by mapnum
        free = find(blk==0);
        robvec = ones(size(free));
        [ri,ci] = find(blk==0);
        G.ri=ri;
        G.ci=ci;
        Moves = repmat( (1:numel(free))',1,4);
        world = -blk;
        world(free) = 1:numel(free);
        for i = 1:numel(free)
            r = ri(i);
            c = ci(i);
            if blk(r,c-1) == 0
                Moves(i,1) = world(r,c-1);
            end
            if blk(r,c+1) == 0
                Moves(i,2) = world(r,c+1);
            end
            if blk(r+1,c) == 0
                Moves(i,3) = world(r+1,c);
            end
            if blk(r-1,c) == 0
                Moves(i,4) = world(r-1,c);
            end
        end
        
    end
end