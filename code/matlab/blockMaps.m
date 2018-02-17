function blk = blockMaps(mapnum)
% blockMaps returns a binary matrix map that have different sizes and
% can be used to read in images as maps selected by mapnum. Each map is 
% described with number of blanks or an image description
% Input: mapnum, the map to be selected
% Output: blk, the binary matrix map
%  Authors:
%  Aaron T. Becker
%     atbecker@uh.edu
%  Arun Mahadev
%     avm.rensol@gmail.com
%  Edited by: Daniel Bao
%     dzbao@uh.edu
% Quick list of maps available
%1-14 blanks
%2-18 blanks
%3-25 blanks
%4-loops with 37 blanks
%5-2 loops, 24 blanks
%6-2 loops, 26 blanks
%7-2 loops, 27 blanks
%8-2 loops, 28 blanks
%9-2 loops, 29 blanks
%10-2 loops, 20 blanks
%11-2 loops 31 blanks
%12-2 loops, 47 blanks
%13-2 loops, 66 blanks
%14-20 blanks; these are mazes not loops
%15-21 blanks
%16-22 blanks
%17-23 blanks
%18-19 blanks
%19-15 blanks
%20-16 blanks
%21-17 blanks
%22-linear map with 50 free spaces
%23-rectangular map with 50 free spaces
%24-%Completely connected and bounded
%image of a small part of a leaf
%25-%Completely connected and bounded
%image of a medium part of a leaf
%26- %Completely connected and bounded
                                    %image of a whole leaf
%27-%Completely connected and bounded
                                         %leaf with 5000 free spaces
%28-%Different leaf with 5000 spaces
%29-%Different leaf with 500 spaces
%30-%Different leaf
%31-%3 level H tree with 1441 spaces
%32-%3 level H tree with 1441 spaces
%33-%4 level H tree with 24193 free spaces
%%%%%%
if mapnum == 1
    blk=uint8([%14 blanks
        1,1,1,1,1,1,1
        1,0,1,1,0,1,1
        1,0,1,1,0,1,1
        1,0,0,0,0,1,1
        1,0,0,0,0,1,1
        1,1,1,1,0,0,1
        1,1,1,1,1,1,1]);
elseif mapnum==2
    blk=uint8([%18 blanks
        1,1,1,1,1,1,1
        1,1,1,0,0,0,1
        1,0,0,0,0,0,1
        1,0,0,1,0,0,1
        1,1,0,1,0,0,1
        1,1,0,0,1,0,1
        1,1,1,1,1,1,1]);
elseif mapnum==3
    blk=uint8([%25 blanks
        1 1 1 1 1 1 1 1 1;
        1 1 1 0 1 0 1 1 1;
        1 1 1 0 0 0 1 1 1;
        1 0 0 1 0 1 0 0 1;
        1 1 0 0 0 0 0 1 1;
        1 0 0 1 0 1 0 0 1;
        1 1 1 0 0 0 1 1 1;
        1 1 1 0 1 0 1 1 1;
        1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==4
    blk=uint8([%loops with 37 blanks
        1 1 1 1 1 1 1 1 1 1 1;
        1 1 1 1 0 0 0 1 1 1 1;
        1 1 1 1 0 1 0 1 1 1 1;
        1 1 1 1 0 0 0 1 1 1 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 0 1 0 0 0 0 0 1 0 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 1 1 1 0 0 0 1 1 1 1;
        1 1 1 1 0 1 0 1 1 1 1;
        1 1 1 1 0 0 0 1 1 1 1;
        1 1 1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==5
    blk=uint8([%2 loops, 24 blanks
        1 1 1 1 1 1 1 1 1 1 1;
        1 1 1 1 0 0 0 1 1 1 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 0 1 0 0 0 0 0 1 0 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 1 1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==6
    blk=uint8([%2 loops, 26 blanks
        1 1 1 1 1 1 1 1 1 1 1;
        1 1 1 1 0 1 0 1 1 1 1;
        1 1 1 1 0 0 0 1 1 1 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 0 1 0 0 0 0 0 1 0 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 1 1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==7
    blk=uint8([%2 loops, 27 blanks
        1 1 1 1 1 1 1 1 1 1 1;
        1 1 1 1 0 1 0 1 1 1 1;
        1 1 1 1 0 0 0 1 1 1 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 0 1 0 0 0 0 0 1 0 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 1 1 1 1 0 1 1 1 1 1;
        1 1 1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==8
    blk=uint8([%2 loops, 28 blanks
        1 1 1 1 1 1 1 1 1 1 1;
        1 1 1 1 0 1 0 1 1 1 1;
        1 1 1 1 0 0 0 1 1 1 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 0 1 0 0 0 0 0 1 0 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 1 1 1 0 0 1 1 1 1 1;
        1 1 1 1 1 1 1 1 1 1 1;
        1 1 1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==9
    blk=uint8([%2 loops, 29 blanks
        1 1 1 1 1 1 1 1 1 1 1;
        1 1 1 1 0 1 0 1 1 1 1;
        1 1 1 1 0 0 0 1 1 1 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 0 1 0 0 0 0 0 1 0 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 1 1 1 0 0 1 1 1 1 1;
        1 1 1 1 0 1 1 1 1 1 1;
        1 1 1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==10
    blk=uint8([%2 loops, 20 blanks   
        1 1 1 1 1 1 1 1 1 1 1;
        1 1 1 1 0 1 0 1 1 1 1;
        1 1 1 1 0 0 0 1 1 1 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 0 1 0 0 0 0 0 1 0 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 1 1 1 0 0 1 1 1 1 1;
        1 1 1 1 0 1 1 1 1 1 1;
        1 1 1 1 0 1 1 1 1 1 1;
        1 1 1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==11
    blk=uint8([%2 loops 31 blanks   
        1 1 1 1 1 1 1 1 1 1 1;
        1 1 1 1 0 1 0 1 1 1 1;
        1 1 1 1 0 0 0 1 1 1 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 0 1 0 0 0 0 0 1 0 1;
        1 0 0 0 1 0 1 0 0 0 1;
        1 1 1 1 0 0 1 1 1 1 1;
        1 1 1 1 0 1 1 1 1 1 1;
        1 1 1 1 0 0 1 1 1 1 1;
        1 1 1 1 1 1 1 1 1 1 1;
        ]);
    
elseif mapnum==12
    blk=uint8([%2 loops, 47 blanks   
        1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
        1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1;
        1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1;
        1 1 1 1 0 1 1 1 1 1 1 1 0 1 1 1 1 1;
        1 1 0 0 0 0 0 1 1 1 0 0 0 0 0 1 1 1;
        1 1 0 1 1 1 0 1 1 1 0 1 0 1 0 1 1 1;
        1 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 1 1;
        1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 1;
        1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==13
    blk=uint8([%2 loops, 66 blanks   
        1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
        1 1 0 0 0 1 1 1 0 1 1 1 0 0 0 0 0 1;
        1 0 0 1 0 0 0 0 0 0 0 0 0 1 1 1 0 1;
        1 0 1 1 0 1 1 0 1 1 1 1 0 1 1 0 0 1;
        1 0 0 0 0 0 0 0 1 1 0 0 0 0 0 1 0 1;
        1 1 0 1 1 1 0 1 1 1 0 1 0 1 0 1 1 1;
        1 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 1 1;
        1 0 1 0 1 0 1 0 0 0 1 0 1 0 1 0 1 1;
        1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==14
    blk=uint8([%20 blanks  
        1 1 1 1 1 1 1 1 1;
        1 1 1 0 1 0 1 1 1;
        1 1 1 0 0 0 1 1 1;
        1 0 0 1 0 1 0 0 1;
        1 1 0 0 0 0 0 1 1;
        1 0 0 1 0 1 0 0 1;
        1 1 1 1 1 1 1 1 1;
        1 1 1 1 1 1 1 1 1;
        1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==15
    blk=uint8([%21 blanks  
        1 1 1 1 1 1 1 1 1;
        1 1 1 0 1 0 1 1 1;
        1 1 1 0 0 0 1 1 1;
        1 0 0 1 0 1 0 0 1;
        1 1 0 0 0 0 0 1 1;
        1 0 0 1 0 1 0 0 1;
        1 1 1 1 0 1 1 1 1;
        1 1 1 1 1 1 1 1 1;
        1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==16
    blk=uint8([%22 blanks  
        1 1 1 1 1 1 1 1 1;
        1 1 1 0 1 0 1 1 1;
        1 1 1 0 0 0 1 1 1;
        1 0 0 1 0 1 0 0 1;
        1 1 0 0 0 0 0 1 1;
        1 0 0 1 0 1 0 0 1;
        1 1 1 0 0 1 1 1 1;
        1 1 1 1 1 1 1 1 1;
        1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==17
    blk=uint8([%23 blanks  
        1 1 1 1 1 1 1 1 1;
        1 1 1 0 1 0 1 1 1;
        1 1 1 0 0 0 1 1 1;
        1 0 0 1 0 1 0 0 1;
        1 1 0 0 0 0 0 1 1;
        1 0 0 1 0 1 0 0 1;
        1 1 1 0 0 1 1 1 1;
        1 1 1 0 1 1 1 1 1;
        1 1 1 1 1 1 1 1 1;
        ]);
elseif mapnum==18
    blk=uint8([%19 blanks
        1,1,1,1,1,1,1 
        1,0,1,0,0,0,1
        1,0,0,0,0,0,1
        1,0,0,1,0,0,1
        1,1,0,1,0,0,1
        1,1,0,0,1,0,1
        1,1,1,1,1,1,1]);
elseif mapnum==19
    blk=uint8([1,1,1,1,1,1,1 
        1,0,0,0,0,1,1
        1,0,1,1,1,1,1
        1,0,0,0,0,1,1
        1,1,1,1,0,1,1
        1,0,0,0,0,0,1
        1,1,1,1,1,1,1]);
elseif mapnum==20
    blk=uint8([%16 blanks
        1,1,1,1,1,1,1 
        1,0,0,0,0,0,1
        1,0,1,1,1,1,1
        1,0,0,0,0,1,1
        1,1,1,1,0,1,1
        1,0,0,0,0,0,1
        1,1,1,1,1,1,1]);
elseif mapnum==21
    blk=uint8([%17 blanks
        1,1,1,1,1,1,1 
        1,0,0,0,0,0,1
        1,0,1,0,1,1,1
        1,0,0,0,0,1,1
        1,1,1,1,0,1,1
        1,0,0,0,0,0,1
        1,1,1,1,1,1,1]);
elseif mapnum==22%linear map with 100 free spaces 
    blk=ones(3,102);
    for i=2:101
        blk(2,i)=0;
    end
elseif mapnum==23%rectangular map with 5000 free spaces 
    blk=zeros(52,102);
    blk(1,:)=1;
    blk(52,:)=1;
    blk(:,1)=1;
    blk(:,102)=1;
elseif mapnum == 24
    blk = imread('leafBWsmall.png');%Completely connected and bounded
                                    %image of a small part of a leaf
    blk=im2bw(blk);
    m2=size(blk,1);
    n2=size(blk,2);
    map_inputz=zeros(m2,n2);
    map_inputz(blk == 1) = 0;
    map_inputz(blk == 0) = 1;
    blk=map_inputz;
elseif mapnum == 25
    blk = imread('leafBWmedium.png');%Completely connected and bounded
                                    %image of a medium part of a leaf
    blk=im2bw(blk);
    m2=size(blk,1);
    n2=size(blk,2);
    map_inputz=zeros(m2,n2);
    map_inputz(blk == 1) = 0;
    map_inputz(blk == 0) = 1;
    blk=map_inputz;
elseif mapnum == 26
    blk = imread('leafBWbig.png');%Completely connected and bounded
                                    %image of a whole leaf
    blk=im2bw(blk);
    m2=size(blk,1);
    n2=size(blk,2);
    map_inputz=zeros(m2,n2);
    map_inputz(blk == 1) = 0;
    map_inputz(blk == 0) = 1;
    blk=map_inputz;
elseif mapnum==27
    blk = imread('leafBWar5000edit.png');%Completely connected and bounded
                                         %leaf with 5000 free spaces
    blk=im2bw(blk);
    m2=size(blk,1);
    n2=size(blk,2);
    map_inputz=zeros(m2,n2);
    map_inputz(blk == 1) = 0;
    map_inputz(blk == 0) = 1;
    blk=map_inputz;
elseif mapnum==28
    blk=imread('leafBWar5000.png');%Different leaf with 5000 spaces
    blk=im2bw(blk);
    m2=size(blk,1);
    n2=size(blk,2);
    map_inputz=zeros(m2,n2);
    map_inputz(blk == 1) = 0;
    map_inputz(blk == 0) = 1;
    blk=map_inputz;
elseif mapnum==29
    blk=imread('leafBWar500.png');%Different leaf with 500 spaces
    blk=im2bw(blk);
    m2=size(blk,1);
    n2=size(blk,2);
    map_inputz=zeros(m2,n2);
    map_inputz(blk == 1) = 0;
    map_inputz(blk == 0) = 1;
    blk=map_inputz;    
elseif mapnum==30
    blk=imread('leafBW.png');%Different leaf
    blk=im2bw(blk);
    m2=size(blk,1);
    n2=size(blk,2);
    map_inputz=zeros(m2,n2);
    map_inputz(blk == 1) = 0;
    map_inputz(blk == 0) = 1;
    blk=map_inputz;
elseif mapnum==31
    blk=imread('HTree_3_1441.png');%3 level H tree with 1441 spaces
    blk=im2bw(blk);
    m2=size(blk,1);
    n2=size(blk,2);
    map_inputz=zeros(m2,n2);
    map_inputz(blk == 1) = 0;
    map_inputz(blk == 0) = 1;
    blk=map_inputz;    
elseif mapnum==32
    blk=imread('HTree_4_5000.png');%4 level H tree with 5000 free spaces
    blk=im2bw(blk);
    m2=size(blk,1);
    n2=size(blk,2);
    map_inputz=zeroes(m2,n2);
    map_inputz(blk == 1) = 0;
    map_inputz(blk == 0) = 1;
    blk=map_inputz;
elseif mapnum==33
    blk=imread('HTree_4_24193');%4 level H tree with 24193 free spaces
    blk=im2bw(blk);
    m2=size(blk,1);
    n2=size(blk,2);
    map_inputz=zeroes(m2,n2);
    map_inputz(blk == 1) = 0;
    map_inputz(blk == 0) = 1;
    blk=map_inputz;
else
    blk=uint8([%Default map with 63 blanks
        1,1,1,1,1,1,1,1,1,1,1
        1,0,1,0,1,1,0,0,0,0,1
        1,0,0,0,0,0,0,0,0,0,1
        1,1,0,1,0,1,0,0,0,0,1
        1,1,0,0,0,1,0,0,0,0,1
        1,1,1,0,1,0,0,0,0,1,1
        1,1,0,0,0,0,0,0,0,0,1
        1,0,0,0,0,1,1,1,0,0,1
        1,1,0,0,0,0,0,0,0,0,1
        1,0,0,0,0,0,0,0,1,0,1
        1,1,1,1,1,1,1,1,1,1,1]);
end
blk = flipud(blk);
end
