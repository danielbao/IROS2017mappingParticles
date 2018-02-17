function [movecount,k,nodecount,init_config] = JanusSpecies(k,itr,max_steps,config_flag,p1,p2,p3,p4,starting_config)
% ClosestFrontier is a demonstration of mapping a completely connected
% and bounded 2D discrete grid space with k particles that move uniformly.
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
% This program was modified with randomized species of each particle that
% behave differently toward the global command.
% The purpose of such a particle is to emulate Janus particles with
% randomized polarization such that their movements are not all uniform
% from the global frame of reference but within their own.
% 
%
% Species used: 
% Species 1-Normal Species; obeys commands perfectly RED
% Species 2-CCW 90 species; goes 90 degrees CCW GREEN
% Species 3-Anti Species; goes 180 degrees in the other direction YELLOW
% Species 4-CW 90 species; goes 90 degrees CW MAGENTA
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
    k = 100;%Default num particles
    itr=1;% Iterations are used for the functionalized version
    config_flag=0; % Whether or not to use the previous configuration
    max_steps=250; % Maximum number of steps we want to take
    starting_config=0; % Inputted configuration
    
end
G.fig = figure(1);
set(gcf,'Renderer','OpenGL');%use OpenGL for graphs, not sure if other
%settings may produce better results
G.mapnum =21;% Identifier for map, 0-26; look at blockMaps to identify each map
G.movecount = 0;%Number of moves made
G.movetyp = [-1,0;0,1;1,0;0,-1];%Array for making moves;
                                %Each row is up, right, left, down
movecount=G.movecount;
G.drawflag=0; % Default 1, draw G.fig on. Set 0 for draw G.fig off.
G.videoflag=0;% Default 0, set to 1 if video is to be made
G.playflag=0;%flag for user playing with keyboard inputs
G.valueflag=0; %flag for user inputting values
G.initflag=1; %flag for first round of initiation; used in updateMap()
clc
%% Making a video demonstration. makemymovie gets the current frame of imge and adds to video file
format compact
MOVIE_NAME =['JanusDemonstration2_map',num2str(G.mapnum),'_',num2str(k),'robots','_video',num2str(itr)]; %Change video name here
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
%% Joystick input code; Uses simulink dependents I think
% if(G.playflag==1)
%     % Define joystick ID (if only using 1 joystick, this will likely be '1')
%     ID = 1;
%     % Create joystick variable
%     joy=vrjoystick(ID);
%     t = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, 'TimerFcn', @t_Callback);
%     start(t);
% end
%% Keyboard input
set(G.fig ,'KeyPressFcn',@keyhandler,'Name','Particle Mapping','color','w')
G.maxX = size(G.obstacle_pos,2);%x-dimension of obstacles in number of columns
G.maxY = size(G.obstacle_pos,1);%y-dimension of obstacles in number of rows
G.colormap = [ 1,1,1; %Empty=white  0
    0.5,0.5,0.5; %undiscovered=grey 1
    1,1,1; %robot=white square with red circle 2
    0,0,0;%obstacle=black 3
    0,0,1;%boundary cells/frontier=blue 4
%     1,1,1;%5
%     1,1,1;%6
%     1,1,1;%7
%     1,1,1;%8
%     1,1,1;%9
%     1,1,1;%10
%     1,1,1;%11
%     1,1,1;%12
%     1,1,1;%13
%     1,1,1;%14
    ];
if config_flag==1%For using the same configurations as before
    G.robvec=starting_config;
else
    randRobots=randperm(numel(G.robvec)); %randomize robots in their positions
%% Distribution of robots code
    G.type1loc=zeros(size(G.robvec));
    G.type2loc=zeros(size(G.robvec));
    G.type3loc=zeros(size(G.robvec));
    G.type4loc=zeros(size(G.robvec));
    G.type1loc(randRobots(1:ceil(k*p1)))=1;
    randRobots(1:ceil(k*p1))=[];    
    G.type2loc(randRobots(1:ceil(k*p2)))=1;
    randRobots(1:ceil(k*p2)-1)=[];
    G.type3loc(randRobots(1:ceil(k*p3)))=1;
    randRobots(1:ceil(k*p3))=[];
    G.type4loc(randRobots(1:ceil(k*p4)))=1;
    %Initialized by the probability distributions of each species
    %4 species locations initialized!!
end
G.robvec=G.type1loc+G.type2loc+G.type3loc+G.type4loc;
init_config=G.robvec; % We store the first locations (linear indices) of the robots
% after they're randomized
RobotVisits=zeros(size(G.obstacle_pos)); %Set blind map to zeros. We want to build path in this map
map_expected=zeros(size(G.obstacle_pos));
map_1=zeros(size(G.obstacle_pos)); %Set zeros initially. We update the expected location of each particle in this map
map_2=zeros(size(G.obstacle_pos));
map_3=zeros(size(G.obstacle_pos));
map_4=zeros(size(G.obstacle_pos));
mapped_obstacles=zeros(size(G.obstacle_pos)); %Map is updated when obstacles are found
frontier_exp= zeros(size(G.obstacle_pos)); %Map to update the locations of frontiers
frontier_exp(1,:)=0;
frontier_exp(3,:)=0;
explored_map= zeros(size(G.obstacle_pos));

axis equal
axis tight
set(gca,'box','off','xTick',[],'ytick',[],'ydir','normal','Visible','on');%Create graph without all of the axes
hold on %This us reqired for proper drawing of the scatter plots; Turning it off will result in deleted plots
updateMap() %Update the map with the information from all the seperate maps
updateTitle() %Update the values displayed in the title
ys=ylim;
ylim([ys(1) ys(2)+0.5]);
for m=1:60%Beginning frames drawn
    makemymovie();
end
%End of initialization
if(G.playflag==0)
    CF() %Closest Frontier mapping call
end
%Closest Frontier mapping call
for m=1:60 %Ending frames result drawn
    makemymovie();
end
%% CF repeatedly moves particles to frontier cells until there are no more frontier cells left
    function CF()
        iter=1;
        while(nnz(frontier_exp)>0&&G.movecount<max_steps)%While there are still unknowns, DFS begins
            frontier_vec=G.boundvec; %Refresh local variable to global current locations of frontiers
            roboloc=G.roboloc; %Refresh local locations to global current locations of robots
            temp1loc=G.ind1;
            temp2loc=G.ind2;
            temp3loc=G.ind3;
            temp4loc=G.ind4;
            moveSeq = BFS_Expansion_AVM_Janus(G.update_map,roboloc,temp1loc,temp2loc,temp3loc,temp4loc,frontier_vec); 
            %The shortest path to a frontier cell is selected by expanding from particles
            steps = max(0,numel(moveSeq));%Get the minimum number of steps from Dijkstra's
            for mvIn =1:steps%Move to the frontier on all particles
                moveto(moveSeq(mvIn));
                nodecount(iter)=length(find(explored_map==0|explored_map==11|explored_map==12|explored_map==13|explored_map==14|explored_map==4));%Update the nodes visited in each step
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
                    if(G.mapnum==2 && (i2==1||i2==3))
                        frontier_exp(i2,j2)==0;
                    end
                else
                    frontier_exp(i2,j2)=0;%It isn't a frontier!
                end
            end
        end
    end


%% Apply move called by moveto to all the robots
    function [rvec1Out, rvec2Out, rvec3Out, rvec4Out]= applyMove(mv, rvec1In, rvec2In,rvec3In,rvec4In)
        %mv=move selected; rvecIn=robots locations, one for each type
        rvec1Out = zeros(size(rvec1In));%Result matrix to store moved robots of type 1
        rvec2Out = zeros(size(rvec2In));
        rvec3Out = zeros(size(rvec3In));
        rvec4Out = zeros(size(rvec4In));
        switch mv
            case 1
                mv2=4;
                mv3=2;
                mv4=3;
            case 2
                mv2=3;
                mv3=1;
                mv4=4;
            case 3
                mv2=1;
                mv3=4;
                mv4=2;
            case 4
                mv2=2;
                mv3=3;
                mv4=1;
            otherwise
                disp('bad mv');
        end
        %1-left
        %2-right
        %3-up
        %4-down?
        %G.Moves returns the index of the free space which would be the
        %result of that move
        %The issue is that sometimes the movements allow the robots to go
        %through the wall or overlap!!
        %Solut
        %We prioritize type 1 movements; they go first;
        if mv==1 || mv==4 %Collision check for left and down
            for ni = 1:numel(rvec1In)%G.Moves(ni,mv) returns 1 if there is a free space after the movement mv
                if rvec1In(G.Moves(ni,mv)) == 0 && rvec2In(G.Moves(ni,mv)) == 0 && rvec3In(G.Moves(ni,mv)) == 0 && rvec4In(G.Moves(ni,mv)) == 0 
                    %If there isn't any species at that location
                    %Move to that location
                    rvec1Out(G.Moves(ni,mv)) = rvec1In(ni);
                    rvec1In(ni)=0;%Clear previous location for robot
                else
                    rvec1Out(ni)=rvec1In(ni);%Keep old location
                end
                rvec1In(ni)=rvec1Out(ni);%Update old info with new location
            end
        else %Collision check for down and right
            for ni=numel(rvec1In):-1:1%For the rest of the robots
                if rvec1In(G.Moves(ni,mv)) == 0 && rvec2In(G.Moves(ni,mv)) == 0 && rvec3In(G.Moves(ni,mv)) == 0 && rvec4In(G.Moves(ni,mv)) == 0
                    %Equivalent to ~=1
                    %If there isn't any robots at that location
                    rvec1Out(G.Moves(ni,mv)) = rvec1In(ni);%Set result to new location
                    rvec1In(ni)=0;%clear input for robot ni
                else
                    rvec1Out(ni)=rvec1In(ni);%Update result with old location
                end
                rvec1In(ni)=rvec1Out(ni);%Set old locations with new location
            end
        end
        %type 2 movement check
        if mv2==1 || mv2==4 %Collision check for left and down
            for ni = 1:numel(rvec2In)%G.Moves(ni,mv) returns 1 if there is a free space after the movement mv
                if rvec1Out(G.Moves(ni,mv2)) == 0 && rvec2In(G.Moves(ni,mv2)) == 0 && rvec3In(G.Moves(ni,mv2)) == 0 && rvec4In(G.Moves(ni,mv2)) == 0
                    %If there isn't a species of type 1,2,3, or 4 that will move to
                    %that location
                    rvec2Out(G.Moves(ni,mv2)) = rvec2In(ni);%Set result to new location
                    rvec2In(ni)=0;%Clear previous location for robot
                else
                    rvec2Out(ni)=rvec2In(ni);%Keep old location
                end
                rvec2In(ni)=rvec2Out(ni);%Update old info with new location
            end
        else %Collision check for down and right
            for ni=numel(rvec2In):-1:1%For the rest of the robots descending
                if rvec1Out(G.Moves(ni,mv2)) == 0 && rvec2In(G.Moves(ni,mv2)) == 0 && rvec3In(G.Moves(ni,mv2)) == 0 && rvec4In(G.Moves(ni,mv2)) == 0
                    %Equivalent to ~=1
                    rvec2Out(G.Moves(ni,mv2)) = rvec2In(ni);%Set result to new location
                    rvec2In(ni)=0;%clear input for robot ni
                else
                    rvec2Out(ni)=rvec2In(ni);%Update result with old location
                end
                rvec2In(ni)=rvec2Out(ni);%Set old locations with new location
            end
        end
        %type 3 collision check
        if mv3==1 || mv3==4 %Collision check for left and down
            for ni = 1:numel(rvec3In)%G.Moves(ni,mv) returns 1 if there is a free space after the movement mv
                if rvec1Out(G.Moves(ni,mv3)) == 0 && rvec2Out(G.Moves(ni,mv3)) == 0 && rvec3In(G.Moves(ni,mv3)) == 0 && rvec4In(G.Moves(ni,mv3)) == 0
                    %If there isn't a species of type 1,2,3, or 4 that will move to
                    %that location
                    rvec3Out(G.Moves(ni,mv3)) = rvec3In(ni);%Set result to new location
                    rvec3In(ni)=0;%Clear previous location for robot
                else
                    rvec3Out(ni)=rvec3In(ni);%Keep old location
                end
                rvec3In(ni)=rvec3Out(ni);%Update old info with new location
            end
        else %Collision check for down and right
            for ni=numel(rvec3In):-1:1%For the rest of the robots descending
                if rvec1Out(G.Moves(ni,mv3)) == 0 && rvec2Out(G.Moves(ni,mv3)) == 0 && rvec3In(G.Moves(ni,mv3)) == 0 && rvec4In(G.Moves(ni,mv3)) == 0
                    %Equivalent to ~=1
                    rvec3Out(G.Moves(ni,mv3)) = rvec3In(ni);%Set result to new location
                    rvec3In(ni)=0;%clear input for robot ni
                else
                    rvec3Out(ni)=rvec3In(ni);%Update result with old location
                end
                rvec1In(ni)=rvec3Out(ni);%Set old locations with new location
            end
        end
        %type 4 collision check
        if mv4==1 || mv4==4 %Collision check for left and down
            for ni = 1:numel(rvec4In)%G.Moves(ni,mv) returns 1 if there is a free space after the movement mv
                if rvec1Out(G.Moves(ni,mv4)) == 0 && rvec2Out(G.Moves(ni,mv4)) == 0 && rvec3Out(G.Moves(ni,mv4)) == 0 && rvec4In(G.Moves(ni,mv4)) == 0
                    %If there isn't a species of type 1,2,3, or 4 that will move to
                    %that location
                    rvec4Out(G.Moves(ni,mv4)) = rvec4In(ni);%Set result to new location
                    rvec1In(ni)=0;%Clear previous location for robot
                else
                    rvec4Out(ni)=rvec4In(ni);%Keep old location
                end
                rvec4In(ni)=rvec4Out(ni);%Update old info with new location
            end
        else %Collision check for down and right
            for ni=numel(rvec4In):-1:1%Starting from the end
                if rvec1Out(G.Moves(ni,mv4)) == 0 && rvec2Out(G.Moves(ni,mv4)) == 0 && rvec3Out(G.Moves(ni,mv4)) == 0 && rvec4In(G.Moves(ni,mv4)) == 0
                    %Equivalent to ~=1
                    rvec4Out(G.Moves(ni,mv4)) = rvec4In(ni);%Set result to new location
                    rvec4In(ni)=0;%clear input for robot ni
                else
                    rvec4Out(ni)=rvec4In(ni);%Update result with old location
                end
                rvec4In(ni)=rvec4Out(ni);%Set old locations with new location
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
%% t_callback processes the joystick input
    function t_Callback(~,~)
        try
            mv=0;%Reset move selector
            Y=axis(joy, 1);     % X-axis is joystick axis 1
            X=axis(joy, 2);     % Y-axis is joystick axis 2
            endflag=button(joy,9); %Pressed Select
            if Y<=-0.1
                mv = 1;
            elseif Y>=0.1
                mv = 2;
            elseif X<=-0.1
                mv = 3;
            elseif X>=0.1
                mv = 4;
            end
            if(endflag>=0.1)
                stop(t);
            end
            movetomv(mv);
        catch
            joy = vrjoystick(1);
            disp('Error');
            return;
        end
    end
%% moveto(key) calls movetomv and updates the map
    function movetomv(mv)
        if mv>0%Do the move as seen from the 
            %global reference frame based on the type given
            type1temp=G.type1loc;
            type2temp=G.type2loc;
            type3temp=G.type3loc;
            type4temp=G.type4loc;
            %These store the locations and performs the respective shifts
            
            map_1=G.im2==1;
            map_2=G.im2==2;
            map_3=G.im2==3;
            map_4=G.im2==4;
            %This gets a logical matrix which we can perform circ shift on
            map_expected=G.im2;%im2 is the local particle location matrix
            %updates every call to updateMap()
            G.movecount = G.movecount+1;%Increment movecount everytime a move is applied
            %Approach to solve this problem
            %Merge the arrays (add them)
            %If they overlap (value>1) then only one of them moves to that
            %new location
            %The one that moves is determined by the type that moves right
            %and up first because those are the ones we check first?
            %i.e we have a global command of left
            %We must move types 3 and 4 first in order to get the right
            %ordering of the particles
            %If there's overlap we can trace backwards to see which
            %particle overlapped into that space and then keep that
            %particle from moving!
            if mv==1
                map_1 = circshift(map_1,[0 -1]);%shift map left by 1
                map_2 = circshift(map_2,[-1 0]);%shift type 2 map down by 1
                map_3 = circshift(map_3,[0 1]);
                map_4 = circshift(map_4,[1 0]);
            elseif mv==2
                map_1 = circshift(map_1,[0 1]);%shift map right by 1
                map_2 = circshift(map_2,[1 0]);
                map_3 = circshift(map_3,[0 -1]);
                map_4 = circshift(map_4,[-1 0]);
            elseif mv==3
                map_1 = circshift(map_1,[1 0]);%shift map up by 1
                map_2 = circshift(map_2,[0 -1]);
                map_3 = circshift(map_3,[-1 0]);
                map_4 = circshift(map_4,[0 1]);
            else
                map_1 = circshift(map_1,[-1 0]);%shift map down by 1
                map_2 = circshift(map_2,[0 1]);
                map_3 = circshift(map_3,[1 0]);
                map_4 = circshift(map_4,[0 -1]);
            end
            
            [G.type1loc, G.type2loc, G.type3loc, G.type4loc]= applyMove(mv, G.type1loc, G.type2loc, G.type3loc, G.type4loc);%Move robots and put it in actual positions
            updateMap()
            updateTitle()
            if G.videoflag==1
                makemymovie()
            end
        end
    end
    function moveto(key)
        %Move selector is from global reference frame
        %Keyboard input
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
           %Previous code just repeated movetomv(), so I just call it here
           %now
           movetomv(mv);
        end
    end
%% Updating the map
    function updateMap()
        current_map = ones(size(G.obstacle_pos)); %1=undiscovered
        current_map(G.free) = 11*G.type1loc + 12*G.type2loc + 13*G.type3loc + 14*G.type4loc; %11=type 1 particles on map
        realmap=flipud(current_map);
        %Each type is marked by 1+ their type number
        G.im2=zeros(size(current_map)); %Set particles matrix to zero. This matrix will have 1 where particles are and zero otherwise
        G.visits=zeros(size(current_map));
        G.im2(G.free(G.type1loc>0))=1;%Set particles matrix to 1 where the robots are
        G.im2(G.free(G.type2loc>0))=2;
        G.im2(G.free(G.type3loc>0))=3;
        G.im2(G.free(G.type4loc>0))=4;
        G.visits(G.free(G.type1loc>0))=1;%Set other particles matrix to 1 where the robots are
        G.visits(G.free(G.type2loc>0))=1;
        G.visits(G.free(G.type3loc>0))=1;
        G.visits(G.free(G.type4loc>0))=1;
        RobotVisits=G.visits+RobotVisits; %RoboVisits stores cells already visited by particles
        %im2 used to store visits but now visits does!
        robIndex = G.type1loc+G.type2loc+G.type3loc+G.type4loc;%Get positions of robots again
        robIndex = find(robIndex);
        nodes(robIndex);
        current_map(RobotVisits==0)=1; %1=undiscovered, set it to undiscovered if there no robot has visited there
        frontier_exp(current_map==11 | current_map ==12 | current_map ==13 | current_map ==14)=0;
        %Current_map is 11,12,13,or 14 when it has been visited; therefore it isn't a frontier
        map_expected(current_map~=1)=0;%If current map is not visited, then there must be nothing expected there
        map_1(current_map~=1)=0;
        map_2(current_map~=1)=0;
        map_3(current_map~=1)=0;
        map_4(current_map~=1)=0;
        mapped_obstacles = mapped_obstacles | map_expected | map_1 | map_2 | map_3 | map_4;%OR the expected and mapped obstacles to update obstacles
        frontier_exp(mapped_obstacles)=0;%all of our obstacles are refreshed to not be frontiers
        current_map(frontier_exp==1)=4; %4=frontier cells
        current_map(mapped_obstacles==1)=3; %3=obstacles
        G.update_map=zeros(size(current_map));%reset update_map
        G.update_map(current_map==3)=1;%Reset our unknowns and obstacles to be undiscovered
        G.update_map(current_map==1)=1;%Reset undiscovered to be undiscovered
        G.boundvec=find(current_map== 4);%Set boundaries to be frontiers
        G.roboloc=find(current_map== 11 | current_map == 12 | current_map == 13 | current_map == 14);
        G.ind1=find(current_map==11);
        G.ind2=find(current_map==12);
        G.ind3=find(current_map==13);
        G.ind4=find(current_map==14);
        %Set robot locations to where they're supposed to be
        [G.type1x,G.type1y]=find(current_map== 11);%Store scatter plot locations of the robot
        [G.type2x,G.type2y]=find(current_map== 12);
        [G.type3x,G.type3y]=find(current_map== 13);
        [G.type4x,G.type4y]=find(current_map== 14);
        [G.Frontx,G.Fronty]=find(current_map== 4);
        %Frontier locations
        [G.Obsx,G.Obsy]=find(current_map== 3);
        %Obstacle locations
        [G.Unkx,G.Unky]=find(current_map== 1);
%         colormap(G.colormap(unique(current_map)+1,:));
        %color map is not needed aymore!
        if G.initflag==1
           %This initialization measure allows us to set the scatterplots for each
           %data set (obstacles, robot locations)
           %Previously refreshing images may be a resource hog; hope
           %fully this approach works better.
%             G.i=scatter(G.Fronty,G.Frontx,'s','b','filled'); 
            %Scatterplot for frontier locations
            G.type1plot=scatter(G.type1y,G.type1x,'o','r','filled'); %Initialize the locations of each particle
            G.type2plot=scatter(G.type2y,G.type2x,'o','g','filled'); 
            G.type3plot=scatter(G.type3y,G.type3x,'o','y','filled'); 
            G.type4plot=scatter(G.type4y,G.type4x,'o','m','filled'); 
            %These are the scatterplots for the robot locations
%             G.j=scatter(G.Obsy,G.Obsx,'s','k','filled');
            %Scatterplot for obstacle locations
%             G.k=scatter(G.Unky,G.Unkx,'s','MarkerFaceColor',[0.5 0.5 0.5],'MarkerEdgeColor',[0.5 0.5 0.5]);
            %Scatterplot for unknown locations
            G.initflag=0;
            currentunits = get(gca,'Units');
            set(gca,'Units','Points');%The points conversion allow for window
            %resizing issues to get solved every draw
            axpos = get(gca,'Position');
            set(gca,'Units', currentunits);
            markerWidth = .8/diff(xlim)*axpos(3); % Calculate Marker width in points
            set(G.type1plot,'SizeData',0.8*markerWidth^2);
            set(G.type2plot,'SizeData',0.8*markerWidth^2);
            set(G.type3plot,'SizeData',0.8*markerWidth^2);
            set(G.type4plot,'SizeData',0.8*markerWidth^2);
%             set(G.i,'SizeData',1.1*markerWidth^2);
%             set(G.j,'SizeData',1.1*markerWidth^2);
%             set(G.k,'SizeData',1.1*markerWidth^2);
            %Reset the flag b/c this is only needed once
        end
        explored_map=current_map;
        tempmap=current_map;
        tempmap(tempmap==11|tempmap==12|tempmap==13|tempmap==14)=2;
        colormap(G.colormap(unique(tempmap)+1,:));
        if G.drawflag==1
            G.axis=imagesc(tempmap); %This really slows drawing down
%             even though it probably requires the least effort in looks
            currentunits = get(gca,'Units');
            set(gca,'Units','Points');%The points conversion allow for window
            %resizing issues to get solved every draw
            axpos = get(gca,'Position');
            set(gca,'Units', currentunits);
            markerWidth = .8/diff(xlim)*axpos(3); % Calculate Marker width in points
%             set(G.i,'XData',G.Fronty,'Ydata',G.Frontx,'SizeData',1.1*markerWidth^2);
%             set(G.j,'XData',G.Obsy,'Ydata',G.Obsx,'SizeData',1.1*markerWidth^2);
%             set(G.k,'XData',G.Unky,'Ydata',G.Unkx,'SizeData',1.1*markerWidth^2);
            set(G.type1plot,'XData',G.type1y,'Ydata',G.type1x,'SizeData',0.8*markerWidth^2);
            set(G.type2plot,'XData',G.type2y,'Ydata',G.type2x,'SizeData',0.8*markerWidth^2);
            set(G.type3plot,'XData',G.type3y,'Ydata',G.type3x,'SizeData',0.8*markerWidth^2);
            set(G.type4plot,'XData',G.type4y,'Ydata',G.type4x,'SizeData',0.8*markerWidth^2);
            uistack(G.type1plot,'top');
            uistack(G.type2plot,'top');
            uistack(G.type3plot,'top');
            uistack(G.type4plot,'top');
            drawnow %limitrate use limit rate for speed up!!
        end
    end
%% updateTitle updates title when called
    function updateTitle()
        if nnz(frontier_exp)==1%Grammatical condition if there is only 1 frontier cell
            FC=' explored cell';
        else
            FC=' explored cells';
        end
        title([num2str(G.movecount), ' moves, ',num2str(k),' particles, ', num2str(length(find(explored_map==0|explored_map==11|explored_map==12|explored_map==13|explored_map==14))), FC,', ', num2str(nnz(G.free)), ' free cells Itr ', num2str(itr)]);
        
    end
%% SetupWorld setups map
    function [blk,free,robvec,Moves] = SetupWorld(mapnum)
        %blk is the position of obstacles
        %free is the index of the free spaces in blk
        %robvec is a binary vector where the ith element is true if there
        %is a robot at free(i).
        %G.Moves just checks if there's a free space next to the given
        %cell, checking has to be done at another level taking into
        %consideration the new locations of the blocks
        blk = blockMaps(mapnum); %Function returns the binary map specified by mapnum
        free = find(blk==0); %Free spaces are where the obstacles are 0
        robvec = ones(size(free)); %Put a robot in every free space
        [ri,ci] = find(blk==0);
        G.ri=ri;
        G.ci=ci;
        Moves = repmat( (1:numel(free))',1,4); %Repeat free four times across
                                               %Represents 4 different move
                                               %combinations possible
        world = -blk;
        %This means 1s are free spaces and 0s are obstacles!
        world(free) = 1:numel(free);%Assign a vector to world at location of free to
        %Represent each possible move at free space
        for i = 1:numel(free)%Iterate through free spaces
            r = ri(i);
            c = ci(i);
            if blk(r,c-1) == 0%Using beginning LR UD conventions, if you can move left
                Moves(i,1) = world(r,c-1);%Store possible left move in Moves from world's left cell
                %This means that it will be 1! 1 means good valid move!
            end
            if blk(r,c+1) == 0%Move right
                Moves(i,2) = world(r,c+1);%Store possible right move in Moves from world's right cell
            end
            if blk(r+1,c) == 0%Move down
                Moves(i,3) = world(r+1,c);%Store possible down move in Moves from world's down cell
            end
            if blk(r-1,c) == 0%Move up
                Moves(i,4) = world(r-1,c);%Store posible up move in Moves from world's down cell
            end
        end
        
    end
end