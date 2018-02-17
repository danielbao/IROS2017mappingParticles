%% Comparison for the total cells explored between data sets
% Do a 1-way ANOVA for the total cell count by the end of the 250th move
% How to perform
% Cycle through the data
% Store the last value of nodecount
% Do an anova, get p-value, be done!
cell_data=zeros(size(janus1temprun,2),2);
for i=100:50:100%size(temprun,1) %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:50%set the numner of iterations for the function
        cell_data(j,1)=janus2temprun(i,j).nodecount(100);
        cell_data(j,2)=janus1temprun(i,j).nodecount(100);

    end
end
[p,tbl,stats] = anova1(cell_data);
[ht,pt] = ttest(cell_data(:,1), cell_data(:,2));
title('Horizontal map exploration for Janusparticles at 200 moves');
ylabel('Total amount of cells explored');
xlabel('Col 1 is 2 Janus Particles; Col 2 is the same particle');