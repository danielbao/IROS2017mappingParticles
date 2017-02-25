function massive3dManipulate
%TODO:  manual manipulation
%TODO:  make one video
close all
format compact

%global FrameCount MOVIE_NAME MAKE_MOVIE
MAKE_MOVIE = 1; % if 1, records png images to make into a movie
MOVIE_NAME = 'massive3dManipulateChairRotate';
%ffmpeg -r 30 -f image2 -i img/massive3dManipulateChair.png -b 24000k -r 30 massive3dManipulateChair.mp4
FrameCount = 0;
set(0,'defaultaxesfontsize',18);
set(0,'defaulttextfontsize',18);

if MAKE_MOVIE
    writerObj = VideoWriter(MOVIE_NAME);%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
    set(writerObj,'Quality',100)
    %set(writerObj, 'CompressionRatio', 2);
    open(writerObj);
end


% show that we can do 3D manipulation even with a Pillar
GoalPts =[...   % a 3D zigzag
    1,1,1;1,2,1;1,3,1; 2,3,1;3,3,1;3,2,1;3,1,1;
    %1,1,2;      1,3,2;           3,1,2;
    3,1,2;
    3,1,3; 2,1,3;1,1,3;1,2,3;1,3,3;2,3,3;3,3,3;
    ];

GoalPts =[...   % a 4x4 table
    1,1,1;  1,4,1;  4,4,1;  4,1,1; %legs
    1,1,2;  1,4,2;  4,4,2;  4,1,2;
    1,1,3;  1,4,3;  4,4,3;  4,1,3;
    1,1,4;1,2,4;1,3,4;1,4,4; %top
    2,1,4;2,2,4;2,3,4;2,4,4;
    3,1,4;3,2,4;3,3,4;3,4,4;
    4,1,4;4,2,4;4,3,4;4,4,4;
    ];

GoalPts =[...   % a 4x4 table
    1,1,1;  1,4,1;  4,4,1;  4,1,1; %legs
    1,1,2;  1,4,2;  4,4,2;  4,1,2;
    1,1,3;  1,4,3;  4,4,3;  4,1,3;
    1,1,4;1,2,4;1,3,4;1,4,4; %top
    2,1,4;2,2,4;2,3,4;2,4,4;
    3,1,4;3,2,4;3,3,4;3,4,4;
    4,1,4;4,2,4;4,3,4;4,4,4;
    4,4,5;  4,1,5; %back
    4,4,6;  4,1,6;
    4,4,7;  4,1,7;
    4,1,8;4,2,8;4,3,8;4,4,8;
    ];



GoalMin = min(GoalPts);
GoalMaxDim = 1+ (max(GoalPts) - GoalMin);
numRobots = size(GoalPts,1);
rs = 1:numRobots;
dimSize = ceil(numRobots^(1/3));
RobotPts = [dimSize-mod(rs-1,dimSize)', mod(floor(( rs-1)/dimSize),dimSize)',floor((rs-1)/dimSize^2)'];
RobotMaxDim = 1 + (max(RobotPts) - min(RobotPts));

GoalXYZ = [1-GoalMin(1),1-GoalMin(2),RobotMaxDim(3)  + GoalMaxDim(3)];
GoalPts = GoalPts+repmat(GoalXYZ,size(GoalPts,1),1);



PillarHt = GoalXYZ(3);
PillarPts = [zeros(PillarHt,1),zeros(PillarHt,1),(1:PillarHt)'];

RobotXYZ = [-RobotMaxDim(1),-RobotMaxDim(2),GoalMaxDim(3)];
RobotPts = RobotPts+repmat(RobotXYZ,size(RobotPts,1),1);
%reorder the goal points;  (the k'th cell arranged in max z, maxy, maxx
GoalPts = sortrows(sortrows(sortrows(GoalPts,-1),-2),-3);

% order the robot points -- min z, maxy, maxx
RobotPts = sortrows(sortrows(sortrows(RobotPts,-1),-2),3);
RobotPts = [RobotPts,(1:size(RobotPts,1))'];  %add an index to the goal points

f1 = figure(1);
vox_sz = [1,1,1]; %length of cube sides
voxel_image(GoalPts, vox_sz,'g',0.1);
hold on
patches = voxel_image(RobotPts(:,1:3), vox_sz,'b',0.5);
voxel_image(PillarPts, vox_sz,'r');

aXmin = -(GoalMaxDim(1) + RobotMaxDim(1) +2);
aXmax =  (GoalMaxDim(1) + RobotMaxDim(1) +2);
aYmin = -(GoalMaxDim(2) + RobotMaxDim(2) +1);
aYmax =  (GoalMaxDim(2) + RobotMaxDim(2) +2);
aZmin = 0.5;
aZmax = 2*( 1+RobotMaxDim(3)  + GoalMaxDim(3));

patch([aXmin,aXmin,aXmax,aXmax],...
    [aYmin,aYmax,aYmax,aYmin],...
    [.5,.5,.5,.5],...
    [.5,.5,.5]);  %floor
axis equal
axis([aXmin,aXmax,aYmin ,aYmax,aZmin,aZmax]);
view([-37.5, 30]);
tixs = (-100:5:100);
set(gca,'XGrid','on','YGrid','on','ZGrid','on','Xtick', tixs,'Ytick', tixs,'Ztick', tixs)
xlabel('x')
ylabel('y')
zlabel('z')
set(f1 ,'color',[1,1,1]);
set(f1 ,'KeyPressFcn',@keyhandler,'Name','Massive Control 3D');
% for k = 1:numRobots  %number the robots and goals
%     text(RobotPts(k,1),RobotPts(k,2),RobotPts(k,3),num2str(RobotPts(k,4)))
%     text(GoalPts(k,1),GoalPts(k,2),GoalPts(k,3),num2str(k))
% end


%automatically move the robots.
for k = 1:numRobots
    move1BoxToGoal(k)
end
for i=1:10
    updateDrawing();
end
for i = 1:2:180
    view([-37.5-i, 30]);
    updateDrawing();
end


title('Finished')

    function keyhandler(src,evnt) %#ok<INUSL>
        moveto(evnt.Key)
    end

    function RobotPt = getRobotPt(k)
        ind = find(RobotPts(:,4)==k,1,'first');
        RobotPt = RobotPts(ind,1:3);
    end

    function move1BoxToGoal(k)
        % determine the box to move -- the max z of the min y of min x
        RobotPt = getRobotPt(k);
        %determine the goal position  (the k'th cell arranged in max z, max
        GoalPt =  GoalPts(k,:);
        DesDelta = GoalPt-RobotPt;
        
        %pop off the bottom row in x direction
        %%% move robots +z
        minZ = min(RobotPts(:,3));
        while minZ <= PillarHt
            moveto('+z');
            RobotPt = getRobotPt(k);
            minZ = min(RobotPts(:,3));
        end
        %%% move under row +y
        while RobotPt(2) <0
            moveto('+y');
            RobotPt = getRobotPt(k);
        end
        %%% move behind row +x
        indRow =  RobotPts(:,2)==RobotPt(2) & RobotPts(:,3)==RobotPt(3);
        minX = min(RobotPts(indRow,1));
        while minX <= 0
            moveto('+x');
            RobotPt = getRobotPt(k);
            indRow =  RobotPts(:,2)==RobotPt(2) & RobotPts(:,3)==RobotPt(3);
            minX = min(RobotPts(indRow,1));
        end
        %%% lower the column -z
        indRow =  RobotPts(:,2)==RobotPt(2);
        minZ = min(RobotPts(indRow,3));
        while minZ > PillarHt
            moveto('-z');
            RobotPt = getRobotPt(k);
            indRow =  RobotPts(:,2)==RobotPt(2);
            minZ = min(RobotPts(indRow,3));
        end
        %%% push row out of the block
        moveto('-x')
        DesDelta = DesDelta-[1,0,0];
        %%% raise the row above the pillar
        indRow =  RobotPts(:,2)==RobotPt(2);
        minZ = min(RobotPts(indRow,3));
        while minZ <= PillarHt
            moveto('+z');
            RobotPt = getRobotPt(k);
            indRow =  RobotPts(:,2)==RobotPt(2);
            minZ = min(RobotPts(indRow,3));
        end
        %%% find the index of the bottom element in the column
        %        indCol = RobotPts(:,1)==RobotPt(1) & RobotPts(:,2)==RobotPt(2);
        %        minZ = min(RobotPts(indCol,3));
        %        colHeight =  RobotPt(3)-minZ;
        %         while minZ <= PillarHt
        %             moveto('+z');
        %             RobotPt = getRobotPt(k);
        %             indCol = RobotPts(:,1)==RobotPt(1) & RobotPts(:,2)==RobotPt(2);
        %             minZ = min(RobotPts(indCol,3));
        %         end
        %%% place robots over column
        while RobotPt(1) >0
            moveto('-x');
            RobotPt = getRobotPt(k);
        end
        %%% push down until robots is above the rest
        %         indCol = RobotPts(:,4)>k;
        %         highestkPlus1 = max(RobotPts(indCol,3));
        %         while RobotPt(3) <= highestkPlus1
        %             moveto('-z');
        %             RobotPt = getRobotPt(k);
        %             indCol = RobotPts(:,4)>k;
        %             highestkPlus1 = max(RobotPts(indCol,3));
        %             DesDelta = DesDelta-[0,0,1];
        %         end
        %         %%% if this is a column, need to push away the column
        %         if colHeight
        %             moveto('-x');
        %             for n = 1:colHeight
        %                 moveto('-z');
        %             end
        %             moveto('+x');
        %         end
        %%% push away the rest (if they exist)
        %         moveto('+x');
        %push box to correct delta y location
        moveto('+y');
        moveto('-z');
        while DesDelta(2) >0
            moveto('-y');
            DesDelta = DesDelta-[0,1,0];
        end
        %push box to correct delta x location
        moveto('+x');
        moveto('-y');
        while DesDelta(1) >0
            moveto('-x');
            DesDelta = DesDelta-[1,0,0];
        end
        %push box to correct delta z location
        moveto('+z');
        moveto('-x');
        while DesDelta(3) >0
            moveto('-z');
            DesDelta = DesDelta-[0,0,1];
        end
        %return to original location
        RobotPt = getRobotPt(k);
        while RobotPt(3) < GoalPt(3)
            moveto('+z');
            RobotPt = getRobotPt(k);
        end
        while RobotPt(2) < GoalPt(2)
            moveto('+y');
            RobotPt = getRobotPt(k);
        end
        while RobotPt(1) < GoalPt(1)
            moveto('+x');
            RobotPt = getRobotPt(k);
        end
        
    end


    function moveto(key)
        % Maps keypresses to moving pixels
        step = [0,0,0];
        if strcmp(key,'leftarrow') || strcmp(key,'-x') %-x
            RobotPts = sortrows(RobotPts,1);
            step = -[1,0,0];
        elseif strcmp(key,'rightarrow')|| strcmp(key,'+x') %+x
            RobotPts = sortrows(RobotPts,-1);
            step = [1,0,0];
        elseif strcmp(key,'uparrow')|| strcmp(key,'+y') %+y
            RobotPts = sortrows(RobotPts,-2);
            step = [0,1,0];
        elseif strcmp(key,'downarrow')|| strcmp(key,'-y') %-y
            RobotPts = sortrows(RobotPts,2);
            step = -[0,1,0];
        elseif strcmp(key,'w')|| strcmp(key,'+z') %+z
            RobotPts = sortrows(RobotPts,-3);
            step = [0,0,1];
        elseif strcmp(key,'s') || strcmp(key,'-z')%-z
            RobotPts = sortrows(RobotPts,3);
            step = -[0,0,1];
        end
        % implement the move on every robot
        for ni = 1:size(RobotPts,1)
            desVal = RobotPts(ni,1:3)+step;
            if  ~ismember(desVal,RobotPts(:,1:3),'rows') && ~ismember(desVal,PillarPts(:,1:3),'rows')...
                    && ~any(desVal<=[aXmin,aYmin,aZmin])&& ~any(desVal>=[aXmax,aYmax,aZmax])
                RobotPts(ni,1:3) = desVal;
            end
        end
        updateVoxel(RobotPts(:,1:3),patches)
        updateDrawing
    end



    function updateVoxel(pts,patches)
        %updates the Voxels with handle patches to be at pts
        np = size(pts,1);
        vert = zeros(8*np,3);
        fac = zeros(6*np,4,'uint32');
        vert_bas = [...
            -0.5,-0.5,-0.5;
            0.5,-0.5,-0.5;
            0.5,0.5,-0.5;
            -0.5,0.5,-0.5;
            -0.5,-0.5,0.5;
            0.5,-0.5,0.5;
            0.5,0.5,0.5;
            -0.5,0.5,0.5];
        vert_bas = vert_bas.*([vox_sz(1).*ones(8,1), vox_sz(2).*ones(8,1), vox_sz(3).*ones(8,1)]);
        fac_bas = [...
            1,2,3,4;
            1,2,6,5;
            2,3,7,6;
            3,4,8,7;
            4,1,5,8;
            5,6,7,8];
        for vx = 1:np
            a = ((vx-1)*8+1):vx*8;
            for dim = 1:3
                vert( a,dim ) = vert_bas(:,dim) + pts(vx,dim);
            end
            fac ( ((vx-1)*6+1):vx*6,: ) = (vx - 1)*8*ones(6,4) + fac_bas;
        end
        set(patches,'Vertices',vert,'Faces',fac);
    end


    function updateDrawing
        %global FrameCount MOVIE_NAME MAKE_MOVIE G
        %set(f1, 'position',[-98          57        1774         965]);
        set(f1, 'position',[81 79 1160 627]);%mac screen width
        drawnow
        if(MAKE_MOVIE)
            FrameCount=FrameCount+1;
            fname = sprintf('img/%s%06d.png',MOVIE_NAME,FrameCount);
            lighting phong
            %set(gcf,'Renderer','zbuffer')
%            lighting gouraud
% set(gcf,'Renderer','OpenGL')
            F = getframe_nosteal_focus; %getframe;
            %imwrite(F.cdata, fname,'png');
            writeVideo(writerObj,F.cdata);
            while(FrameCount < 10)
                updateDrawing
            end
            F = 1;
            %clear F
        end
    end
end

