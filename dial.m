function [count1 count2]=dial(theta,OD_pair,O,D,W,OD,linkweight)
%求起点到终点最短路
for i=1:max(O)
    for j=1:max(O)
        [dist,path]=graphshortestpath(linkweight,i,j);%最短路工具箱
        dis(i,j)=dist;
    end
end
r=dis(1,:);%节点1到任何一节点的最短路
s=dis(:,max(O));%任何一点到节点j的最短路
%定义发点
for i=1:max(O)
    m=find(O==i);
    Oi{i}=[D(m)];
end
%定义收点
%for i=1:9
for i=1:max(O)
    m=find(D==i);
    Di{i}=[O(m)];
end
Di{1}=1;
%输入起点和终点

%%正向流
count_positive=sparse(O,D,zeros(1,length(O)));
for i=1:length(OD_pair)-1
    origin=OD_pair(i);
    for j=(i+1):length(OD_pair)
        %origin=2;
        %destination=5;
        destination=OD_pair(j);
        r=dis(origin,:);
        s=dis(:,destination)';
        Q=OD(i,j);
        Lsparse=likelihood(r,s,theta,W,O,D);
        Wi=weight2(r,Lsparse,Oi,Di,origin,O,D,destination);
        Xi=flow2(s,O,D,Oi,Di,Wi,destination,Q,origin);
        count_positive=count_positive+Xi;
    end
end

%%反向流
count_negative=sparse(O,D,zeros(1,length(O)));
for i=1:length(OD_pair)-1
    origin=OD_pair(i);
    for j=(i+1):length(OD_pair)
        %origin=2;
        %destination=5;
        destination=OD_pair(j);
        r=dis(origin,:);
        s=dis(:,destination)';
        Q=OD(j,i);
        Lsparse=likelihood(r,s,theta,W,O,D);
        Wi=weight2(r,Lsparse,Oi,Di,origin,O,D,destination);
        Xi=flow2(s,O,D,Oi,Di,Wi,destination,Q,origin);
        count_negative=count_negative+Xi;
    end
end
count1=full(count_positive);
count2=full(count_negative)';
end