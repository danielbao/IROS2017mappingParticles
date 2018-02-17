%Function to run experiments multiple times
function multi_runJanus()
run= struct;
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

for i=10:100:4000 %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:1 %set the numner of iterations for the function
        [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus2temprun(i,j).init_config] = JanusSpecies(i,j,Inf,0,0.5,0,0.5,0,0); 
        clf
        %specify which function is used. Currently it will run the random mapping
        [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config] = JanusSpecies(i,j,Inf,1,1,0,0,0,janus2temprun(i,j).init_config); 
        clf
        save('Janus2_1D_5000_temp.mat','janus2temprun'); %We save this to get nodecount plot later
        save('Janus1_1D_5000_temp.mat','janus1temprun');
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
