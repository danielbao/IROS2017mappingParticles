%Function to run experiments multiple times
function multi_runJanus()
janus4temprun=struct;
janus3temprun=struct;
janus2temprun=struct;
janus1temprun=struct;
%  i=1;
%
%  for j=1:10
%     [temprun(j).movecount,temprun(j).k,temprun(j).nodecount] = bluedijkstra_v2_1_18(i);
%  end
%      run(i).movecount=mean([temprun.movecount]);
%      run(i).k=mean([temprun.k]);
%      run(i).nodecount=mean([temprun.nodecount]);

for i=100:20:400 %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:25 %set the numner of iterations for the function
        [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config] = JanusSpecies(i,j,Inf,0,1,0,0,0,0); 
        %specify which function is used. Currently it will run the random mapping
        [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus2temprun(i,j).init_config] = JanusSpecies(i,j,Inf,1,0.5,0,0.5,0,janus1temprun(i,j).init_config); 
        [janus3temprun(i,j).movecount,janus3temprun(i,j).k,janus3temprun(i,j).nodecount,janus3temprun(i,j).init_config] = JanusSpecies(i,j,Inf,1,0.33,0.33,0.33,0.34,janus1temprun(i,j).init_config); 
        [janus4temprun(i,j).movecount,janus4temprun(i,j).k,janus4temprun(i,j).nodecount,janus4temprun(i,j).init_config] = JanusSpecies(i,j,Inf,1,0.25,0.25,0.25,0.25,janus1temprun(i,j).init_config); 
        
        clf
        save('Janus2_2D_100_corrected_temp.mat','janus2temprun'); %We save this to get nodecount plot later
        save('Janus1_2D_100_corrected_temp.mat','janus1temprun');
    end
%     run(i).movecount=mean([temprun(i,:).movecount]);
%     run(i).stderr=std([temprun(i,:).movecount]);
%     run(i).k=mean([temprun(i,:).k]);
end
% save('JanusQuick5000.mat','run');
% figure;
% shadedErrorBar([run.k],[run.movecount],[run.stderr],'g');
% xlabel('Number of Particles (n)');
% ylabel('Number of Moves (k)');
% figure;


end
