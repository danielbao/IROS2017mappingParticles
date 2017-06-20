function [movecount,k,nodecount] = bluedijkstra_elect(k,itr)
% Allows user to use arrow keys to move robots uniformly to search a grey
% space. Obstacles appear as black when we detect them.
%TODO:
%1. Automated DFS with termination condition. [done]
%2. keep track of how many unknown boundary cells exist -- the DFS should
%be along these boundary cells, not
%  Aaron T. Becker
%     atbecker@uh.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global G;
if nargin<1
    k = 300;%37;%numRobots
    itr=1;
end
G.fig = figure(1);
G.mapnum = 24;%22;
G.movetyp = [-1,0;0,1;1,0;0,-1];
G.alg = 4;
G.numCommands = 0;
G.totalMoves = 0;
G.sizeSteps = 1000;
G.uniquePos = ones(G.sizeSteps,1);
G.step = 0;
G.movecount = 0;
clc
format compact
MOVIE_NAME = 'elect_101'; %to make a movie 'Small_particles_collection.mp4'
G.fig = figure(1);
clf
writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
set(writerObj,'Quality',100);
writerObj.FrameRate=30;
open(writerObj);
    function makemymovie()% each frame has to be added. So a function is made and called whenever an image needs to be added
        figure(G.fig)
        F = getframe(G.fig);
        writeVideo(writerObj,F.cdata);
    end

[G.obstacle_pos,G.free,G.robvec,G.Moves] = SetupWorld(G.mapnum);
set(G.fig ,'KeyPressFcn',@keyhandler,'Name','Massive Control','color','w')
G.movecount = 0;
G.goal=[];
G.EMPTY = 0;
G.OBST = 1;

G.maxX = size(G.obstacle_pos,2);
G.maxY = size(G.obstacle_pos,1);

figure(1)
clf

G.colormap = [  1,1,1; %Empty = white  0
    0.5,0.5,0.5; %undiscovered= grey 1
    1,1,1; %robot= white square with red circle 2
    0,0,0;%obstacle= black           3
    0,0,1;%boundary cells= blue      4
    %     0,1,0;%target frontier=green     5
    ];

randRobots=randperm(numel(G.robvec)); %randRobots: 1:num_empty_spaces, randomly arranged
G.robvec(randRobots(k+1:end))=0; % locations of the k robots

RobotVisits=zeros(size(G.obstacle_pos)); %Set blind map to zeros. We want to build path in this map
map_expected=zeros(size(G.obstacle_pos));
mapped_obstacles=zeros(size(G.obstacle_pos));
bound_exp= zeros(size(G.obstacle_pos));
updateMap()
set(gca,'box','off','xTick',[],'ytick',[],'ydir','normal','Visible','on');
axis equal
axis tight
updateTitle()
%   makemymovie()
hold on
%initialising one of the exisiting robots as the leader
loc=randsample(numel(G.roboloc),1);
roboloc=G.roboloc(loc); % index location of the lead robot in the workspace
list=find(G.robvec);
moveloc=list(loc); %index of lead robot among all the free spaces
[G.leadx,G.leady]=ind2sub(size(G.update_map),roboloc); %x,y posotion of lead robot
%moveto(1)
% DFS('1'); %-x
% DFS('2');%+x
% DFS('3');%+y
% DFS('4');%-y

drawcirc()
% pause()
%  pause
boundpush()

movecount=G.movecount;

    function nodes(robIndex) %todo: optimize this function
        %for each robot, check if the cell in direction mv is unknown
        %           [i,j]=ind2sub(size(G.obstacle_pos),robIndex);
        for mv_type=1:4
            %[I,J] = ind2sub(robIndex,G.obstacle_pos)
            for c = 1:numel(robIndex)
                i2 = G.ri(robIndex(c))+G.movetyp(mv_type,2);
                j2 = G.ci(robIndex(c))+G.movetyp(mv_type,1);
                % if the cell has never been visited and isn't an obstacle
                if RobotVisits(i2,j2) == 0 && mapped_obstacles(i2,j2) == 0
                    bound_exp(i2,j2)=1;
                else
                    bound_exp(i2,j2)=0;
                end
            end
        end
    end

    function boundpush()
        iter=1;
        %           loc=randsample(numel(G.roboloc),1);
        %             roboloc=G.roboloc(loc);
        
        while(nnz(bound_exp)>0)
            
            boundvec=G.boundvec;
            [moveSeq,G.goal] = DijkstraForBoundary_mod(G.update_map,roboloc,boundvec);
            %              steps = min(inf,numel(moveSeq));
            %              for mvIn =1:steps
            %                  G.step = G.step +1;
            % moveloc is the location of the lead robot's next
            % position in the list of free spaces. We move one step at
            % a time and find the closest node in each step. G.bound
            % vec and roboloc changes in each move
            %                if strcmp(moveSeq(1),'l')
            %                     mv=1;
            %                 elseif strcmp(moveSeq(1),'r')
            %                     mv=2;
            %                 elseif strcmp(moveSeq(1),'u')
            %                     mv=3;
            %                 elseif strcmp(moveSeq(1),'d')
            %                     mv=4;
            %                end
            %                 moveloc=G.Moves(moveloc,mv);
            for j=1:numel(moveSeq)
%                 if G.goal
%                     [goalxy(1,1),goalxy(1,2)]=ind2sub(size(G.obstacle_pos),G.goal);
%                     scatter(goalxy(1,2),goalxy(1,1),'filled','g');
%                     drawnow();
%                     if G.movecount==0 || G.movecount==50
%                         pause
%                     end
%                              makemymovie();
%                 end
                moveto(moveSeq(j));
                if G.boundvec~=G.goal
                    break
                elseif nnz(bound_exp)==0
                    break
                end
            end
            %end
            %function call to implement the move sequence
            
            nodecount(iter)=nnz(bound_exp);
            iter=iter+1;
            %makemymovie()
            %end
            
            
            %end DFS
        end
        for i=1:5
            %makemymovie()
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
                    if ni==moveloc
                        moveloc=G.Moves(ni,mv);
                    end
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
                    if ni==moveloc
                        moveloc=G.Moves(ni,mv);
                    end
                else
                    rvec2(ni)=rvecIn(ni);
                end
            end
        end
        G.leadx=G.ri(moveloc);G.leady=G.ci(moveloc);
        roboloc=sub2ind(size(G.update_map),G.leadx,G.leady);
        
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
        % Maps keypresses to moving pixels
        %         if strcmp(key,'r')  %RESET
        %             [G.obstacle_pos,G.free,G.robvec,G.Moves] = SetupWorld(G.mapnum);
        %             return
        %         end
        
        mv=0;
        if strcmp(key,'leftarrow') || strcmp(key,'l')|| strcmp(key,'1') %-x
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
            %develop a expectation map where we set a matrix based on moves
            %the map has 1 where robot is expected to be
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
            drawcirc()
            
            drawnow
            %                          makemymovie()
        end
    end
%% Drawing scatter circles the size of matrix cell rather than in point size
    function drawcirc()
        h=scatter(G.robscaty,G.robscatx,'r','filled');
%         if G.goal
%                     [goalxy(1,1),goalxy(1,2)]=ind2sub(size(G.obstacle_pos),G.goal);
%                     scatter(goalxy(1,2),goalxy(1,1),'filled','g');
%                     %          makemymovie();
%         end
        l=scatter(G.leady,G.leadx,'g','filled');% Create a scatter plot and return a handle to the 'hggroup' object
        drawGoal();
        %Obtain the axes size (in axpos) in Points
        currentunits = get(gca,'Units');
        set(gca, 'Units', 'Points');
        axpos = get(gca,'Position');
        set(gca, 'Units', currentunits);
        markerWidth = .8/diff(xlim)*axpos(3); % Calculate Marker width in points
        set(h,'SizeData', markerWidth^2)
        set(l,'SizeData', markerWidth^2)
        
        drawnow
    end
%% Drawing the Goal
    function drawGoal()
        if G.goal
            [goalxy(1,1),goalxy(1,2)]=ind2sub(size(G.obstacle_pos),G.goal);
            scatter(goalxy(1,2),goalxy(1,1),'filled','g');
            
            drawnow();
            if G.movecount==0 || G.movecount==50
                %pause
            end
            %          makemymovie();
        end
    end
%% Updating the displayed image map
    function updateMap() %
        
        im = ones(size(G.obstacle_pos)); % 1== undiscovered
        
        im(G.free) = 2*G.robvec; %2 = robots draw robots on map
        
        G.im2=zeros(size(im)); %Set robots matrix to zero. This matrix will have 1 where robots are and zero otherwise
        G.im2(G.free(G.robvec>0))=1;
        RobotVisits=G.im2+RobotVisits; %im3 tracks the number of times robots have been in each cell
        robIndex = find(G.robvec);
        nodes(robIndex);
        
        im(RobotVisits==0)=1; % 1== undiscovered
        bound_exp(im==2)=0;
        map_expected(im~=1)=0; % 0 = free space
        mapped_obstacles = mapped_obstacles | map_expected;
        bound_exp(mapped_obstacles)=0;
        im(bound_exp==1)=4;
        
        
        G.update_map=zeros(size(im));
        G.update_map(im==3)=1;
        G.update_map(im==1)=1;
        G.boundvec=find(im== 4);
        
        
        G.roboloc =find(im== 2);
        im(mapped_obstacles==1)=3; % 3 = obstacles
        [G.robscatx,G.robscaty]=find(im== 2);
        colormap(G.colormap(unique(im)+1,:));
        G.axis=imagesc(im); %show map that updates as robots explore
        
    end
    function updateTitle()
        if nnz( bound_exp)==1
            FC=' Frontier Cell';
        else
            FC=' Frontier Cells';
        end
        title([num2str(G.movecount), ' moves, ',num2str(sum(G.robvec)),' particles, ', num2str(nnz( bound_exp)), FC,', Iteration :', num2str(itr)])
    end
%% setup map
    function [blk,free,robvec,Moves] = SetupWorld(mapnum)
        %  blk is the position of obstacles
        % free is the index of the free spaces in blk
        % robvec is a binary vector where the ith element is true if there
        % is a robot at free(i).
        if mapnum == 500
            blk = imread('HTree_2_73.png');
            blk= im2bw(blk);
            m2=size(blk,1);
            n2=size(blk,2);
            map_inputz=zeros(m2,n2);
            map_inputz(blk == 1) = 0;
            map_inputz(blk == 0) = 1;
            blk=map_inputz;
        elseif mapnum==700
            blk = imread('HTree_3_1441.png');
            blk= im2bw(blk);
            m2=size(blk,1);
            n2=size(blk,2);
            map_inputz=zeros(m2,n2);
            map_inputz(blk == 1) = 0;
            map_inputz(blk == 0) = 1;
            blk=map_inputz;
        elseif mapnum==6000
            blk = imread('leafBWar5000edit.png');
            blk= im2bw(blk);
            m2=size(blk,1);
            n2=size(blk,2);
            map_inputz=zeros(m2,n2);
            map_inputz(blk == 1) = 0;
            map_inputz(blk == 0) = 1;
            blk=map_inputz;
        else
            blk = blockMaps(mapnum);
        end
        free = uint16(find(blk==0));
        robvec = ones(size(free));
        [ri,ci] = find(blk==0);
        G.ri=ri;
        G.ci=ci;
        %Mu gives the index in robvec after applying an up move.
        Moves = repmat( (1:numel(free))',1,4);
        world = -blk;
        world(free) = 1:numel(free);
        % hardcode mapping: if I move up, what does that map to in world?
        % I could do this as a vector multiplication
        %l,r,u,d
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