run1=struct;
run2=struct;
for i=100:50:900 %set the range of values for the function. For now it will run for no of robots=500,2000
%     for j=1:50 %set the numner of iterations for the function
%         [janus2temprun(i,j).movecount,janus2temprun(i,j).k,janus2temprun(i,j).nodecount,janus2temprun(i,j).init_config] = JanusSpecies(i,j,250,0,0.5,0,0.5,0,0); 
%         clf
%         %specify which function is used. Currently it will run the random mapping
%         [janus1temprun(i,j).movecount,janus1temprun(i,j).k,janus1temprun(i,j).nodecount,janus1temprun(i,j).init_config] = JanusSpecies(i,j,250,1,1,0,0,0,janus2temprun(i,j).init_config); 
%         clf
%         save('Janus2_1D_100_temp.mat','janus2temprun'); %We save this to get nodecount plot later
%         save('Janus1_1D_100_temp.mat','janus1temprun');
%     end
    run1(i).movecount=mean([janus1temprun(i,:).movecount]);
    run1(i).stderr=std([janus1temprun(i,:).movecount]);
    run1(i).k=mean([janus1temprun(i,:).k]);
    run2(i).movecount=mean([janus2temprun(i,:).movecount]);
    run2(i).stderr=std([janus2temprun(i,:).movecount]);
    run2(i).k=mean([janus2temprun(i,:).k]);
end
figure;
hold on;
shadedErrorBar([run1.k],[run1.movecount],[run1.stderr],'g');
shadedErrorBar([run2.k],[run2.movecount],[run2.stderr],'b');
alpha 0.5;
title('1D Simulation for Janus Particles on 100 space Linear Map');
xlabel('Number of Particles (n)');
ylabel('Number of Moves (k)');
h=findobj(gca,'Type','Line');
legend([h(1) h(4)],'1 Janus Species', '2 Janus Species');
%Instructions of getting a good legend for the right colors
% h=findobj(gca,'Type','Line')
% Depending on how you plotted it, the first 3 should be the first
% plot, the 2nd 3 should be the 2nd plot
% Call legend(h(lineindex),'NameOfLine'); for each line!