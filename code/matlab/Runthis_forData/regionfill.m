function robvec=regionfill(map,k,index)
% free=find(map==0);
% start=randsample(numel(free),1);
% start=free(start);
start=index
[starti,startj]=ind2sub(size(map),start);
[freei,freej]=ind2sub(size(map),free);
% distmap=zeros(1,2);
for i=1:numel(free)
        X = [starti,startj;freei(i),freej(i)];
        d= pdist(X,'euclidean');
        distmap(i,1)=d;
        distmap(i,2)=free(i);
        
end
distsort=sortrows(distmap,1);
list=distsort(1:k,2);
map(list)=2;
[m,n]=size(map);
robvec=reshape(map,m*n,1);
remove=find(robvec==1);
robvec(remove)=[];
robvec(robvec==2)=1;
