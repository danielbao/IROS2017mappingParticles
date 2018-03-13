%Function to run experiments multiple times
function multi_runJanus()
janus2temprun2=struct;
janus2temprun3=struct;
janus2temprun4=struct;
janus3temprun=struct;
janus3temprun2=struct;
janus2temprun=struct;
janus1temprun=struct;
janus4temprun=struct;
%  i=1;
%
%  for j=1:10
%     [temprun(j).movecount,temprun(j).k,temprun(j).nodecount] = bluedijkstra_v2_1_18(i);
%  end
%      run(i).movecount=mean([temprun.movecount]);
%      run(i).k=mean([temprun.k]);
%      run(i).nodecount=mean([temprun.nodecount]);
%Leaf5000
% for i=100:100:4000 %set the range of values for the function. For now it will run for no of robots=500,2000
%     for j=1:25 %set the numner of iterations for the function
%             %                                                                                                      JanusSpecies(k,itr,max_steps,map,config_flag,fill_flag,fill,p1,p2,p3,p4,starting_config)
% 
%         [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus1temprun(i,j).fillseed] = JanusSpecies(i,j,50000,27,0,1,0,1,0,0,0,0); 
%         %This is the control
%         %specify which function is used. Currently it will run the random mapping
%         [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus2temprun(i,j).init_config,janus2temprun(i,j).fillseed] = JanusSpecies(i,j,50000,27,1,1,0,0.5,0,0.5,0,janus1temprun(i,j).init_config); 
%         %Typical 180 rotation type
%         [janus3temprun(i,j).movecount,janus3temprun(i,j).k,janus3temprun(i,j).nodecount,janus3temprun(i,j).init_config,janus3temprun(i,j).fillseed] = JanusSpecies(i,j,50000,27,1,1,0,0.33,0.33,0.34,0,janus1temprun(i,j).init_config); 
%         %180 rotation and 90 rotation
%         [janus2temprun2(i,j).movecount,janus2temprun2(i,j).k,janus2temprun2(i,j).nodecount,janus2temprun2(i,j).init_config,janus2temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,27,1,1,0,0,0.5,0,0.5,janus1temprun(i,j).init_config); 
%         %90 degree offset CCW and 180 degree, type 2 and 4
%         [janus2temprun3(i,j).movecount,janus2temprun3(i,j).k,janus2temprun3(i,j).nodecount,janus2temprun3(i,j).init_config,janus2temprun3(i,j).fillseed] = JanusSpecies(i,j,50000,27,1,1,0,0.5,0,0,0.5,janus1temprun(i,j).init_config); 
%         %Typical rotation and 90 CW; type 1 and 4
%         [janus2temprun4(i,j).movecount,janus2temprun4(i,j).k,janus2temprun4(i,j).nodecount,janus2temprun4(i,j).init_config,janus2temprun4(i,j).fillseed] = JanusSpecies(i,j,50000,27,1,1,0,0.5,0.5,0,0,janus1temprun(i,j).init_config); 
%         %Type 1 and 2
%         [janus3temprun2(i,j).movecount,janus3temprun2(i,j).k,janus3temprun2(i,j).nodecount,janus3temprun2(i,j).init_config,janus3temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,27,1,1,0,0,0.33,0.34,0.33,janus1temprun(i,j).init_config); 
%         %Everything but the type 1
%         [janus4temprun(i,j).movecount,janus4temprun(i,j).k,janus4temprun(i,j).nodecount,janus4temprun(i,j).init_config,janus4temprun(i,j).fillseed] = JanusSpecies(i,j,50000,27,1,1,0,0.25,0.25,0.25,0.25,janus1temprun(i,j).init_config); 
% 
%         clf
%         save('Janus2_2D_Leaf5000_corrected_temp.mat','janus2temprun'); %We save this to get nodecount plot later
%         save('Janus1_2D_Leaf5000_corrected_temp.mat','janus1temprun');
%         save('Janus2_2D_Leaf5000_corrected_temp2.mat','janus2temprun2');
%         save('Janus2_2D_Leaf5000_corrected_temp3.mat','janus2temprun3');
%         save('Janus2_2D_Leaf5000_corrected_temp4.mat','janus2temprun4');
%         save('Janus3_2D_Leaf5000_corrected_temp.mat','janus3temprun');
%         save('Janus3_2D_Leaf5000_corrected_temp2.mat','janus3temprun2');
%         save('Janus4_2D_Leaf5000_corrected_temp.mat','janus4temprun');
%         
%     end
% %     run(i).movecount=mean([temprun(i,:).movecount]);
% %     run(i).stderr=std([temprun(i,:).movecount]);
% %     run(i).k=mean([temprun(i,:).k]);
% end
%HTree 5000
mapnum=31;
janus2temprun2=struct;
janus2temprun3=struct;
janus2temprun4=struct;
janus3temprun=struct;
janus3temprun2=struct;
janus2temprun=struct;
janus1temprun=struct;
janus4temprun=struct;
for i=100:100:4000 %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:25 %set the numner of iterations for the function
        [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus1temprun(i,j).fillseed] = JanusSpecies(i,j,50000,mapnum,0,1,0,1,0,0,0,0); 
        %This is the control
        %specify which function is used. Currently it will run the random mapping
        [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus2temprun(i,j).init_config,janus2temprun(i,j).fillseed] = JanusSpecies(i,j,50000,mapnum,1,1,0,0.5,0,0.5,0,janus1temprun(i,j).init_config); 
        %Typical 180 rotation type
        [janus3temprun(i,j).movecount,janus3temprun(i,j).k,janus3temprun(i,j).nodecount,janus3temprun(i,j).init_config,janus3temprun(i,j).fillseed] = JanusSpecies(i,j,50000,mapnum,1,1,0,0.33,0.33,0.34,0,janus1temprun(i,j).init_config); 
        %180 rotation and 90 rotation
        [janus2temprun2(i,j).movecount,janus2temprun2(i,j).k,janus2temprun2(i,j).nodecount,janus2temprun2(i,j).init_config,janus2temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,mapnum,1,1,0,0,0.5,0,0.5,janus1temprun(i,j).init_config); 
        %90 degree offset CCW and 180 degree, type 2 and 4
        [janus2temprun3(i,j).movecount,janus2temprun3(i,j).k,janus2temprun3(i,j).nodecount,janus2temprun3(i,j).init_config,janus2temprun3(i,j).fillseed] = JanusSpecies(i,j,50000,mapnum,1,1,0,0.5,0,0,0.5,janus1temprun(i,j).init_config); 
        %Typical rotation and 90 CW; type 1 and 4
        [janus2temprun4(i,j).movecount,janus2temprun4(i,j).k,janus2temprun4(i,j).nodecount,janus2temprun4(i,j).init_config,janus2temprun4(i,j).fillseed] = JanusSpecies(i,j,50000,mapnum,1,1,0,0.5,0.5,0,0,janus1temprun(i,j).init_config); 
        %Type 1 and 2
        [janus3temprun2(i,j).movecount,janus3temprun2(i,j).k,janus3temprun2(i,j).nodecount,janus3temprun2(i,j).init_config,janus3temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,mapnum,1,1,0,0,0.33,0.34,0.33,janus1temprun(i,j).init_config); 
        %Everything but the type 1
        [janus4temprun(i,j).movecount,janus4temprun(i,j).k,janus4temprun(i,j).nodecount,janus4temprun(i,j).init_config,janus4temprun(i,j).fillseed] = JanusSpecies(i,j,50000,mapnum,1,1,0,0.25,0.25,0.25,0.25,janus1temprun(i,j).init_config); 

        clf
        save('Janus2_2D_HTree5000_corrected_temp.mat','janus2temprun'); %We save this to get nodecount plot later
        save('Janus1_2D_HTree5000_corrected_temp.mat','janus1temprun');
        save('Janus2_2D_HTree5000_corrected_temp2.mat','janus2temprun2');
        save('Janus2_2D_HTree5000_corrected_temp3.mat','janus2temprun3');
        save('Janus2_2D_HTree5000_corrected_temp4.mat','janus2temprun4');
        save('Janus3_2D_HTree5000_corrected_temp.mat','janus3temprun');
        save('Janus3_2D_HTree5000_corrected_temp2.mat','janus3temprun2');
        save('Janus4_2D_HTree5000_corrected_temp.mat','janus4temprun');
    end
end
%Rectangle 5000
janus2temprun2=struct;
janus2temprun3=struct;
janus2temprun4=struct;
janus3temprun=struct;
janus3temprun2=struct;
janus2temprun=struct;
janus1temprun=struct;
janus4temprun=struct;
for i=100:100:4000 %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:25 %set the numner of iterations for the function
        [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus1temprun(i,j).fillseed] = JanusSpecies(i,j,50000,23,0,1,0,1,0,0,0,0); 
        %This is the control
        %specify which function is used. Currently it will run the random mapping
        [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus2temprun(i,j).init_config,janus2temprun(i,j).fillseed] = JanusSpecies(i,j,50000,23,1,1,0,0.5,0,0.5,0,janus1temprun(i,j).init_config); 
        %Typical 180 rotation type
        [janus3temprun(i,j).movecount,janus3temprun(i,j).k,janus3temprun(i,j).nodecount,janus3temprun(i,j).init_config,janus3temprun(i,j).fillseed] = JanusSpecies(i,j,50000,23,1,1,0,0.33,0.33,0.34,0,janus1temprun(i,j).init_config); 
        %180 rotation and 90 rotation
        [janus2temprun2(i,j).movecount,janus2temprun2(i,j).k,janus2temprun2(i,j).nodecount,janus2temprun2(i,j).init_config,janus2temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,23,1,1,0,0,0.5,0,0.5,janus1temprun(i,j).init_config); 
        %90 degree offset CCW and 180 degree, type 2 and 4
        [janus2temprun3(i,j).movecount,janus2temprun3(i,j).k,janus2temprun3(i,j).nodecount,janus2temprun3(i,j).init_config,janus2temprun3(i,j).fillseed] = JanusSpecies(i,j,50000,23,1,1,0,0.5,0,0,0.5,janus1temprun(i,j).init_config); 
        %Typical rotation and 90 CW; type 1 and 4
        [janus2temprun4(i,j).movecount,janus2temprun4(i,j).k,janus2temprun4(i,j).nodecount,janus2temprun4(i,j).init_config,janus2temprun4(i,j).fillseed] = JanusSpecies(i,j,50000,23,1,1,0,0.5,0.5,0,0,janus1temprun(i,j).init_config); 
        %Type 1 and 2
        [janus3temprun2(i,j).movecount,janus3temprun2(i,j).k,janus3temprun2(i,j).nodecount,janus3temprun2(i,j).init_config,janus3temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,23,1,1,0,0,0.33,0.34,0.33,janus1temprun(i,j).init_config); 
        %Everything but the type 1
        [janus4temprun(i,j).movecount,janus4temprun(i,j).k,janus4temprun(i,j).nodecount,janus4temprun(i,j).init_config,janus4temprun(i,j).fillseed] = JanusSpecies(i,j,50000,23,1,1,0,0.25,0.25,0.25,0.25,janus1temprun(i,j).init_config); 

        clf
        save('Janus2_2D_Rect5000_corrected_temp.mat','janus2temprun'); %We save this to get nodecount plot later
        save('Janus1_2D_Rect5000_corrected_temp.mat','janus1temprun');
        save('Janus2_2D_Rect5000_corrected_temp2.mat','janus2temprun2');
        save('Janus2_2D_Rect5000_corrected_temp3.mat','janus2temprun3');
        save('Janus2_2D_Rect5000_corrected_temp4.mat','janus2temprun4');
        save('Janus3_2D_Rect5000_corrected_temp.mat','janus3temprun');
        save('Janus3_2D_Rect5000_corrected_temp2.mat','janus3temprun2');
        save('Janus4_2D_Rect5000_corrected_temp.mat','janus4temprun');
    end
end
%Leaf 500
janus2temprun2=struct;
janus2temprun3=struct;
janus2temprun4=struct;
janus3temprun=struct;
janus3temprun2=struct;
janus2temprun=struct;
janus1temprun=struct;
janus4temprun=struct;
for i=10:20:400 %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:25 %set the numner of iterations for the function
        [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus1temprun(i,j).fillseed] = JanusSpecies(i,j,50000,29,0,1,0,1,0,0,0,0); 
        %This is the control
        %specify which function is used. Currently it will run the random mapping
        [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus2temprun(i,j).init_config,janus2temprun(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,1,0,0.5,0,0.5,0,janus1temprun(i,j).init_config); 
        %Typical 180 rotation type
        [janus3temprun(i,j).movecount,janus3temprun(i,j).k,janus3temprun(i,j).nodecount,janus3temprun(i,j).init_config,janus3temprun(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,1,0,0.33,0.33,0.34,0,janus1temprun(i,j).init_config); 
        %180 rotation and 90 rotation
        [janus2temprun2(i,j).movecount,janus2temprun2(i,j).k,janus2temprun2(i,j).nodecount,janus2temprun2(i,j).init_config,janus2temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,1,0,0,0.5,0,0.5,janus1temprun(i,j).init_config); 
        %90 degree offset CCW and 180 degree, type 2 and 4
        [janus2temprun3(i,j).movecount,janus2temprun3(i,j).k,janus2temprun3(i,j).nodecount,janus2temprun3(i,j).init_config,janus2temprun3(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,1,0,0.5,0,0,0.5,janus1temprun(i,j).init_config); 
        %Typical rotation and 90 CW; type 1 and 4
        [janus2temprun4(i,j).movecount,janus2temprun4(i,j).k,janus2temprun4(i,j).nodecount,janus2temprun4(i,j).init_config,janus2temprun4(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,1,0,0.5,0.5,0,0,janus1temprun(i,j).init_config); 
        %Type 1 and 2
        [janus3temprun2(i,j).movecount,janus3temprun2(i,j).k,janus3temprun2(i,j).nodecount,janus3temprun2(i,j).init_config,janus3temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,1,0,0,0.33,0.34,0.33,janus1temprun(i,j).init_config); 
        %Everything but the type 1
        [janus4temprun(i,j).movecount,janus4temprun(i,j).k,janus4temprun(i,j).nodecount,janus4temprun(i,j).init_config,janus4temprun(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,1,0,0.25,0.25,0.25,0.25,janus1temprun(i,j).init_config); 

        clf
        save('Janus2_2D_Leaf500_corrected_temp.mat','janus2temprun'); %We save this to get nodecount plot later
        save('Janus1_2D_Leaf500_corrected_temp.mat','janus1temprun');
        save('Janus2_2D_Leaf500_corrected_temp2.mat','janus2temprun2');
        save('Janus2_2D_Leaf500_corrected_temp3.mat','janus2temprun3');
        save('Janus2_2D_Leaf500_corrected_temp4.mat','janus2temprun4');
        save('Janus3_2D_Leaf500_corrected_temp.mat','janus3temprun');
        save('Janus3_2D_Leaf500_corrected_temp2.mat','janus3temprun2');
        save('Janus4_2D_Leaf500_corrected_temp.mat','janus4temprun');
    end
end
%HTree 500
janus2temprun2=struct;
janus2temprun3=struct;
janus2temprun4=struct;
janus3temprun=struct;
janus3temprun2=struct;
janus2temprun=struct;
janus1temprun=struct;
janus4temprun=struct;
for i=10:20:400 %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:25 %set the numner of iterations for the function
        [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus1temprun(i,j).fillseed] = JanusSpecies(i,j,50000,31,0,1,0,1,0,0,0,0); 
        %This is the control
        %specify which function is used. Currently it will run the random mapping
        [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus2temprun(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,1,0,0.5,0,0.5,0,janus1temprun(i,j).init_config); 
        %Typical 180 rotation type
        [janus3temprun(i,j).movecount,janus3temprun(i,j).k,janus3temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus3temprun(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,1,0,0.33,0.33,0.34,0,janus1temprun(i,j).init_config); 
        %180 rotation and 90 rotation
        [janus2temprun2(i,j).movecount,janus2temprun2(i,j).k,janus2temprun2(i,j).nodecount,janus2temprun2(i,j).init_config,janus2temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,0,1,0,0.5,0,0.5,janus1temprun(i,j).init_config); 
        %90 degree offset CCW and 180 degree, type 2 and 4
        [janus2temprun3(i,j).movecount,janus2temprun3(i,j).k,janus2temprun3(i,j).nodecount,janus2temprun3(i,j).init_config,janus2temprun3(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,1,0,0.5,0,0,0.5,janus1temprun(i,j).init_config); 
        %Typical rotation and 90 CW; type 1 and 4
        [janus2temprun4(i,j).movecount,janus2temprun4(i,j).k,janus2temprun4(i,j).nodecount,janus2temprun4(i,j).init_config,janus2temprun4(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,1,0,0.5,0.5,0,0,janus1temprun(i,j).init_config); 
        %Type 1 and 2
        [janus3temprun2(i,j).movecount,janus3temprun2(i,j).k,janus3temprun2(i,j).nodecount,janus3temprun2(i,j).init_config,janus3temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,1,0,0,0.33,0.34,0.33,janus1temprun(i,j).init_config); 
        %Everything but the type 1
        [janus4temprun(i,j).movecount,janus4temprun(i,j).k,janus4temprun(i,j).nodecount,janus4temprun(i,j).init_config,janus4temprun(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,1,0,0.25,0.25,0.25,0.25,janus1temprun(i,j).init_config); 

        clf
        save('Janus2_2D_HTree500_corrected_temp.mat','janus2temprun'); %We save this to get nodecount plot later
        save('Janus1_2D_HTree500_corrected_temp.mat','janus1temprun');
        save('Janus2_2D_HTree500_corrected_temp2.mat','janus2temprun2');
        save('Janus2_2D_HTree500_corrected_temp3.mat','janus2temprun3');
        save('Janus2_2D_HTree500_corrected_temp4.mat','janus2temprun4');
        save('Janus3_2D_HTree500_corrected_temp.mat','janus3temprun');
        save('Janus3_2D_HTree500_corrected_temp2.mat','janus3temprun2');
        save('Janus4_2D_HTree500_corrected_temp.mat','janus4temprun');
    end
end
%Rectangle 500
janus2temprun2=struct;
janus2temprun3=struct;
janus2temprun4=struct;
janus3temprun=struct;
janus3temprun2=struct;
janus2temprun=struct;
janus1temprun=struct;
janus4temprun=struct;
for i=10:20:400 %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:25 %set the numner of iterations for the function
        [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus1temprun(i,j).fillseed] = JanusSpecies(i,j,50000,231,0,1,0,1,0,0,0,0); 
        %This is the control
        %specify which function is used. Currently it will run the random mapping
        [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus2temprun(i,j).init_config,janus2temprun(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,1,0,0.5,0,0.5,0,janus1temprun(i,j).init_config); 
        %Typical 180 rotation type
        [janus3temprun(i,j).movecount,janus3temprun(i,j).k,janus3temprun(i,j).nodecount,janus3temprun(i,j).init_config,janus3temprun(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,1,0,0.33,0.33,0.34,0,janus1temprun(i,j).init_config); 
        %180 rotation and 90 rotation
        [janus2temprun2(i,j).movecount,janus2temprun2(i,j).k,janus2temprun2(i,j).nodecount,janus2temprun2(i,j).init_config,janus2temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,1,0,0,0.5,0,0.5,janus1temprun(i,j).init_config); 
        %90 degree offset CCW and 180 degree, type 2 and 4
        [janus2temprun3(i,j).movecount,janus2temprun3(i,j).k,janus2temprun3(i,j).nodecount,janus2temprun3(i,j).init_config,janus2temprun3(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,1,0,0.5,0,0,0.5,janus1temprun(i,j).init_config); 
        %Typical rotation and 90 CW; type 1 and 4
        [janus2temprun4(i,j).movecount,janus2temprun4(i,j).k,janus2temprun4(i,j).nodecount,janus2temprun4(i,j).init_config,janus2temprun4(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,1,0,0.5,0.5,0,0,janus1temprun(i,j).init_config); 
        %Type 1 and 2
        [janus3temprun2(i,j).movecount,janus3temprun2(i,j).k,janus3temprun2(i,j).nodecount,janus3temprun2(i,j).init_config,janus3temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,1,0,0,0.33,0.34,0.33,janus1temprun(i,j).init_config); 
        %Everything but the type 1
        [janus4temprun(i,j).movecount,janus4temprun(i,j).k,janus4temprun(i,j).nodecount,janus4temprun(i,j).init_config,janus4temprun(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,1,0,0.25,0.25,0.25,0.25,janus1temprun(i,j).init_config); 

        clf
        save('Janus2_2D_Rect500_corrected_temp.mat','janus2temprun'); %We save this to get nodecount plot later
        save('Janus1_2D_Rect500_corrected_temp.mat','janus1temprun');
        save('Janus2_2D_Rect500_corrected_temp2.mat','janus2temprun2');
        save('Janus2_2D_Rect500_corrected_temp3.mat','janus2temprun3');
        save('Janus2_2D_Rect500_corrected_temp4.mat','janus2temprun4');
        save('Janus3_2D_Rect500_corrected_temp.mat','janus3temprun');
        save('Janus3_2D_Rect500_corrected_temp2.mat','janus3temprun2');
        save('Janus4_2D_Rect500_corrected_temp.mat','janus4temprun');
    end
end
%Bolus simulation Rectangle 500
janus2temprun2=struct;
janus2temprun3=struct;
janus2temprun4=struct;
janus3temprun=struct;
janus3temprun2=struct;
janus2temprun=struct;
janus1temprun=struct;
janus4temprun=struct;
for i=10:20:400 %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:25 %set the numner of iterations for the function
        [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus1temprun(i,j).fillseed] = JanusSpecies(i,j,50000,231,0,2,0,1,0,0,0,0); 
        %This is the control
        %specify which function is used. Currently it will run the random mapping
        [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus2temprun(i,j).init_config,janus2temprun(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,2,janus1temprun(i,j).fillseed,0.5,0,0.5,0,janus1temprun(i,j).init_config); 
        %Typical 180 rotation type
        [janus3temprun(i,j).movecount,janus3temprun(i,j).k,janus3temprun(i,j).nodecount,janus3temprun(i,j).init_config,janus3temprun(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,2,janus1temprun(i,j).fillseed,0.33,0.33,0.34,0,janus1temprun(i,j).init_config); 
        %180 rotation and 90 rotation
        [janus2temprun2(i,j).movecount,janus2temprun2(i,j).k,janus2temprun2(i,j).nodecount,janus2temprun2(i,j).init_config,janus2temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,2,janus1temprun(i,j).fillseed,0,0.5,0,0.5,janus1temprun(i,j).init_config); 
        %90 degree offset CCW and 180 degree, type 2 and 4
        [janus2temprun3(i,j).movecount,janus2temprun3(i,j).k,janus2temprun3(i,j).nodecount,janus2temprun3(i,j).init_config,janus2temprun3(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,2,janus1temprun(i,j).fillseed,0.5,0,0,0.5,janus1temprun(i,j).init_config); 
        %Typical rotation and 90 CW; type 1 and 4
        [janus2temprun4(i,j).movecount,janus2temprun4(i,j).k,janus2temprun4(i,j).nodecount,janus2temprun4(i,j).init_config,janus2temprun4(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,2,janus1temprun(i,j).fillseed,0.5,0.5,0,0,janus1temprun(i,j).init_config); 
        %Type 1 and 2
        [janus3temprun2(i,j).movecount,janus3temprun2(i,j).k,janus3temprun2(i,j).nodecount,janus3temprun2(i,j).init_config,janus3temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,2,janus1temprun(i,j).fillseed,0,0.33,0.34,0.33,janus1temprun(i,j).init_config); 
        %Everything but the type 1
        [janus4temprun(i,j).movecount,janus4temprun(i,j).k,janus4temprun(i,j).nodecount,janus4temprun(i,j).init_config,janus4temprun(i,j).fillseed] = JanusSpecies(i,j,50000,231,1,2,janus1temprun(i,j).fillseed,0.25,0.25,0.25,0.25,janus1temprun(i,j).init_config); 

        clf
        save('Janus2_2D_Rect500_bolus_temp.mat','janus2temprun'); %We save this to get nodecount plot later
        save('Janus1_2D_Rect500_bolus_temp.mat','janus1temprun');
        save('Janus2_2D_Rect500_bolus_temp2.mat','janus2temprun2');
        save('Janus2_2D_Rect500_bolus_temp3.mat','janus2temprun3');
        save('Janus2_2D_Rect500_bolus_temp4.mat','janus2temprun4');
        save('Janus3_2D_Rect500_bolus_temp.mat','janus3temprun');
        save('Janus3_2D_Rect500_bolus_temp2.mat','janus3temprun2');
        save('Janus4_2D_Rect500_bolus_temp.mat','janus4temprun');
    end
end
%Leaf 500 bolus
janus2temprun2=struct;
janus2temprun3=struct;
janus2temprun4=struct;
janus3temprun=struct;
janus3temprun2=struct;
janus2temprun=struct;
janus1temprun=struct;
janus4temprun=struct;
for i=10:20:400 %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:25 %set the numner of iterations for the function
        [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus1temprun(i,j).fillseed] = JanusSpecies(i,j,50000,29,0,2,0,1,0,0,0,0); 
        %This is the control
        %specify which function is used. Currently it will run the random mapping
        [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus2temprun(i,j).init_config,janus2temprun(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,2,janus1temprun(i,j).fillseed,0.5,0,0.5,0,janus1temprun(i,j).init_config); 
        %Typical 180 rotation type
        [janus3temprun(i,j).movecount,janus3temprun(i,j).k,janus3temprun(i,j).nodecount,janus3temprun(i,j).init_config,janus3temprun(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,2,janus1temprun(i,j).fillseed,0.33,0.33,0.34,0,janus1temprun(i,j).init_config); 
        %180 rotation and 90 rotation
        [janus2temprun2(i,j).movecount,janus2temprun2(i,j).k,janus2temprun2(i,j).nodecount,janus2temprun2(i,j).init_config,janus2temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,2,janus1temprun(i,j).fillseed,0,0.5,0,0.5,janus1temprun(i,j).init_config); 
        %90 degree offset CCW and 180 degree, type 2 and 4
        [janus2temprun3(i,j).movecount,janus2temprun3(i,j).k,janus2temprun3(i,j).nodecount,janus2temprun3(i,j).init_config,janus2temprun3(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,2,janus1temprun(i,j).fillseed,0.5,0,0,0.5,janus1temprun(i,j).init_config); 
        %Typical rotation and 90 CW; type 1 and 4
        [janus2temprun4(i,j).movecount,janus2temprun4(i,j).k,janus2temprun4(i,j).nodecount,janus2temprun4(i,j).init_config,janus2temprun4(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,2,janus1temprun(i,j).fillseed,0.5,0.5,0,0,janus1temprun(i,j).init_config); 
        %Type 1 and 2
        [janus3temprun2(i,j).movecount,janus3temprun2(i,j).k,janus3temprun2(i,j).nodecount,janus3temprun2(i,j).init_config,janus3temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,2,janus1temprun(i,j).fillseed,0,0.33,0.34,0.33,janus1temprun(i,j).init_config); 
        %Everything but the type 1
        [janus4temprun(i,j).movecount,janus4temprun(i,j).k,janus4temprun(i,j).nodecount,janus4temprun(i,j).init_config,janus4temprun(i,j).fillseed] = JanusSpecies(i,j,50000,29,1,2,janus1temprun(i,j).fillseed,0.25,0.25,0.25,0.25,janus1temprun(i,j).init_config); 

        clf
        save('Janus2_2D_Leaf500_bolus_temp.mat','janus2temprun'); %We save this to get nodecount plot later
        save('Janus1_2D_Leaf500_bolus_temp.mat','janus1temprun');
        save('Janus2_2D_Leaf500_bolus_temp2.mat','janus2temprun2');
        save('Janus2_2D_Leaf500_bolus_temp3.mat','janus2temprun3');
        save('Janus2_2D_Leaf500_bolus_temp4.mat','janus2temprun4');
        save('Janus3_2D_Leaf500_bolus_temp.mat','janus3temprun');
        save('Janus3_2D_Leaf500_bolus_temp2.mat','janus3temprun2');
        save('Janus4_2D_Leaf500_bolus_temp.mat','janus4temprun');
    end
end
%HTree 500
janus2temprun2=struct;
janus2temprun3=struct;
janus2temprun4=struct;
janus3temprun=struct;
janus3temprun2=struct;
janus2temprun=struct;
janus1temprun=struct;
janus4temprun=struct;
for i=10:20:400 %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:25 %set the numner of iterations for the function
        [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus1temprun(i,j).fillseed] = JanusSpecies(i,j,50000,31,0,2,0,1,0,0,0,0); 
        %This is the control
        %specify which function is used. Currently it will run the random mapping
        [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus2temprun(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,2,janus1temprun(i,j).fillseed,0.5,0,0.5,0,janus1temprun(i,j).init_config); 
        %Typical 180 rotation type
        [janus3temprun(i,j).movecount,janus3temprun(i,j).k,janus3temprun(i,j).nodecount,janus1temprun(i,j).init_config,janus3temprun(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,2,janus1temprun(i,j).fillseed,0.33,0.33,0.34,0,janus1temprun(i,j).init_config); 
        %180 rotation and 90 rotation
        [janus2temprun2(i,j).movecount,janus2temprun2(i,j).k,janus2temprun2(i,j).nodecount,janus2temprun2(i,j).init_config,janus2temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,2,janus1temprun(i,j).fillseed,0,0.5,0,0.5,janus1temprun(i,j).init_config); 
        %90 degree offset CCW and 180 degree, type 2 and 4
        [janus2temprun3(i,j).movecount,janus2temprun3(i,j).k,janus2temprun3(i,j).nodecount,janus2temprun3(i,j).init_config,janus2temprun3(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,2,janus1temprun(i,j).fillseed,0.5,0,0,0.5,janus1temprun(i,j).init_config); 
        %Typical rotation and 90 CW; type 1 and 4
        [janus2temprun4(i,j).movecount,janus2temprun4(i,j).k,janus2temprun4(i,j).nodecount,janus2temprun4(i,j).init_config,janus2temprun4(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,2,janus1temprun(i,j).fillseed,0.5,0.5,0,0,janus1temprun(i,j).init_config); 
        %Type 1 and 2
        [janus3temprun2(i,j).movecount,janus3temprun2(i,j).k,janus3temprun2(i,j).nodecount,janus3temprun2(i,j).init_config,janus3temprun2(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,2,janus1temprun(i,j).fillseed,0,0.33,0.34,0.33,janus1temprun(i,j).init_config); 
        %Everything but the type1
        [janus4temprun(i,j).movecount,janus4temprun(i,j).k,janus4temprun(i,j).nodecount,janus4temprun(i,j).init_config,janus4temprun(i,j).fillseed] = JanusSpecies(i,j,50000,31,1,2,janus1temprun(i,j).fillseed,0.25,0.25,0.25,0.25,janus1temprun(i,j).init_config); 

        clf
        save('Janus2_2D_HTree500_bolus_temp.mat','janus2temprun'); %We save this to get nodecount plot later
        save('Janus1_2D_HTree500_bolus_temp.mat','janus1temprun');
        save('Janus2_2D_HTree500_bolus_temp2.mat','janus2temprun2');
        save('Janus2_2D_HTree500_bolus_temp3.mat','janus2temprun3');
        save('Janus2_2D_HTree500_bolus_temp4.mat','janus2temprun4');
        save('Janus3_2D_HTree500_bolus_temp.mat','janus3temprun');
        save('Janus3_2D_HTree500_bolus_temp2.mat','janus3temprun2');
        save('Janus4_2D_HTree500_bolus_temp.mat','janus4temprun');
    end
end
end

