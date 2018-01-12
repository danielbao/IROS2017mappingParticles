function [pathL,goalf] = DijkstraForBoundary_mod_Janus(Graph, source, source1, source2, source3, source4, goal)
%DijkstraForBoundary_mod is a shortest distance function which returns pathL ` 
%Graph is a binary array, with 0 for every white, red, or blue square and
% 1 for everything else
% source is a list of the coordinates on Graph that are red (robots)
% Goal is a list of the coordinates of Graph that are blue (frontiers)
%  Authors: Arun Mahadev & Aaron T. Becker, based on
% https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
% This is actually more of a BFS now!
% No identification pursued here; it will be searched from the path
% back-tracking!
Graph=flipud(Graph);
Q = find(Graph==0); % create vertex set Q from the available spaces to navigate
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
u_not=0; %storing unflipped value to determine the type of source cell the
%path is taken from
type=0; %type of source cell for path
lowvalflag=0;
while numel(Q)>0
    
    [~,ind] = min(dist(Q)); % Source node will be selected first from the indices
                            % where the graph can be traversed
                            % We are selecting type 1 robots to explore
                            % first, but ultimately we want the shortest
                            % path
    dirs = [ 0,-1;% left
        -1,0; % up
        0,1;  % right
        1,0 ; ];  % down
    u = Q(ind);
    Q(ind) = []; % remove u from Q
    dirLetter = ['l','u','r','d'];
    [ui,uj] = ind2sub(size(Graph),u); % Get row column index of u
    for i = 1:size(dirs,1) % for each neighbor v of u: where v is still in Q.
                           % Navigates through each direction next the source
        v = sub2ind(size(Graph), dirs(i,1) + ui,  dirs(i,2) +uj);
        %Get the row column index of the target node to explore
        if Graph(v) == 0 %only try to move if the vertex is 0
            alt = 1+dist(u); %distance to node v if we come from node u
                             %Note that dist(u) is zero because it is the
                             %source node
            if alt < dist(v) %Compares it to infinity, will probably always
                             %be true
                dist(v) = alt;%set that distance
                prevL(v) = dirLetter(i);% get the move sequence
                prev(v) = u;% previous indice traversed is v
                % find if we have reached a goal node is v in goal.If so,
                % return path
               if any(goal==v)%If a frontier is our currently explored node
                    lowvalflag=1;%We go and get a path!
               end
%% Path determining
               if lowvalflag==1%If it's a frontier
                   %
                   goalf = v;
                   path = zeros(1,dist(v));
                   pathL = repmat(' ',1,dist(v));
                   path(dist(v)) = prev(v);
                   pathL(dist(v)) = prevL(v);
                   for k = dist(v)-1:-1:1
                       path(k) = prev(path(k+1));
                       pathL(k) =prevL(path(k+1));
                   end
                   dist(u)=-1;
                   u_not=find(flipud(dist)==-1);
                   type=find(u_not==source1);
                   if(isempty(type))
                       type=find(u_not==source2);
                   end
                   if(isempty(type))
                       type=find(u_not==source3);
                   end
                   if(isempty(type))
                       type=find(u_not==source4);
                   end
                   %Find the type of original cell u was
                   switch type
                       case 1
                           %We don't need to do anything
                       case 2
                           for i=1:numel(pathL)
                               switch pathL(i)
                                   case 'l'
                                       pathL(i)='u';
                                   case 'u'
                                       pathL(i)='r';
                                   case 'r'
                                       pathL(i)='d';
                                   case 'd'
                                       pathL(i)='l';
                                   otherwise
                                       
                               end
                           end
                       case 3
                           for i=1:numel(pathL)
                               switch pathL(i)
                                   case 'l'
                                       pathL(i)='r';
                                   case 'u'
                                       pathL(i)='d';
                                   case 'r'
                                       pathL(i)='l';
                                   case 'd'
                                       pathL(i)='u';
                                   otherwise
                                       
                               end
                           end                           
                       case 4
                           for i=1:numel(pathL)
                               switch pathL(i)
                                   case 'l'
                                       pathL(i)='d';
                                   case 'u'
                                       pathL(i)='l';
                                   case 'r'
                                       pathL(i)='u';
                                   case 'd'
                                       pathL(i)='r';
                                   otherwise   
                               end
                           end                           
                       otherwise
                           disp('Bad type');
                   end
                       
                   Q = [];
                   break;
               end
            end
        end
    end  
end
%% Finding location of goal frontier cell to be returned
goalbackmat=zeros(size(goalmat));
    goalbackmat(goalf)=1;
    goalbackmat=flipud(goalbackmat);
    goalf=find(goalbackmat);

end