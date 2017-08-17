% This code will try to plot the frontier cells to the number of moves
hold on
for i=100:100:size(temprun,1) %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:size(temprun,2)%set the numner of iterations for the function
        tempnodecount=temprun(i,j).nodecount;
        plot(tempnodecount,'r');
    end
end