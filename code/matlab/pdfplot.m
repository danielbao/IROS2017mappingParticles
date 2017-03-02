
%open your figure

%

%set(0,'defaultaxesfontsize',18);
%set(0,'defaulttextfontsize',18);
%set(0,'DefaultTextInterpreter', 'latex')
set(gca,'Position',[0.17 0.20 0.772857 0.715])
set(gcf,'PaperUnits','inches')
set(gcf,'papersize',[7,5])
set(gcf,'paperposition',[0,0,7,5])

print -dpdf 'Mapcomp.pdf'
