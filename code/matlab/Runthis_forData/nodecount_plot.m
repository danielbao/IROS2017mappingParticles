% This code will try to plot the frontier cells to the number of moves
hold on
for i=100:50:100%size(temprun,1) %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:size(janustemprun,2)%set the numner of iterations for the function
        tempnodecount=janustemprun(i,j).nodecount;
        plot(tempnodecount,'b');
    end
end
for i=100:50:100%size(temprun,1) %set the range of values for the function. For now it will run for no of robots=500,2000
    for j=1:size(temprun,2)%set the numner of iterations for the function
        tempnodecount=temprun(i,j).nodecount;
        plot(tempnodecount,'g');
    end
end
title('Total explored spaces for 100 robots in leaf map');
xlabel('Moves');
ylabel('Total explored spaces at each move');
alpha(0.5); 