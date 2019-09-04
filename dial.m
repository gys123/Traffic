function [count1 count2]=dial(theta,OD_pair,O,D,W,OD,linkweight)
%����㵽�յ����·
for i=1:max(O)
    for j=1:max(O)
        [dist,path]=graphshortestpath(linkweight,i,j);%���·������
        dis(i,j)=dist;
    end
end
r=dis(1,:);%�ڵ�1���κ�һ�ڵ�����·
s=dis(:,max(O));%�κ�һ�㵽�ڵ�j�����·
%���巢��
for i=1:max(O)
    m=find(O==i);
    Oi{i}=[D(m)];
end
%�����յ�
%for i=1:9
for i=1:max(O)
    m=find(D==i);
    Di{i}=[O(m)];
end
Di{1}=1;
%���������յ�

%%������
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

%%������
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