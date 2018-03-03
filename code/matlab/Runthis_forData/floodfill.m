function robvec=floodfill(map,k)
free=find(map==0);
start=randsample(numel(free),1);
start=free(start);
[i,j]=ind2sub(size(map),start);
e=sub2ind(size(map),i,j+1);
w=sub2ind(size(map),i,j-1);
no=sub2ind(size(map),i-1,j);
s=sub2ind(size(map),i+1,j);
if map(start)==0
    queue(1)=start;
    iter=1;
    while numel(queue)>0 && iter<=k
        list(iter)=queue(1);
        if map(e)==0
            map(e)=2;
      queue=[queue e];
      iter=iter+1;
      list(iter)=e;
        end
        if map(w)==0
            map(w)=2;
      queue=[queue w];
      iter=iter+1;
      list(iter)=w;
        end
         if map(no)==0
             map(no)=2;
      queue=[queue no];
     iter=iter+1;
     list(iter)=no;
         end
         if map(s)==0
             map(s)=2;
      queue=[queue s];
      iter=iter+1;
      list(iter)=s;
         end  
         queue(1)=[];
         if numel(queue)>0
      [i,j]=ind2sub(size(map),queue(1));
e=sub2ind(size(map),i,j+1);
w=sub2ind(size(map),i,j-1);
no=sub2ind(size(map),i-1,j);
s=sub2ind(size(map),i+1,j);
         end
    end
else
    error('Start position is not a free space');
end
k=min(iter,k);
list=list(1:k);
map(list)=2;
[m,n]=size(map);
robvec=reshape(map,m*n,1);
remove=find(robvec==1);
robvec(remove)=[];
robvec(robvec==2)=1;
