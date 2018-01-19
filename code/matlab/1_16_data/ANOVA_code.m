%% Comparison for the total cells explored between data sets
% Do a 1-way ANOVA for the total cell count by the end of the 250th move
% How to perform
% Cycle through the data
% Store the last value of nodecount
% Do an anova, get p-value, be done!
cell_data=zeros(size(janustemprun,2),2);
for i=100:50:100%size(temprun,1) %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:size(janustemprun,2)%set the numner of iterations for the function
        cell_data(j,1)=janustemprun(i,j).nodecount(250);
    end
end
for i=100:50:100%size(temprun,1) %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:size(temprun,2)%set the numner of iterations for the function
        cell_data(j,2)=temprun(i,j).nodecount(250);
    end
end
[p,tbl,stats] = anova1(cell_data);