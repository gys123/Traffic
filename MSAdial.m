% Guyangsong 07/13/2018  
% ��������Ҫ�õ�matlab����ͼ������
%% �����ʼ��Ϣ
 %O=[1,1,2,2,3,4,4,5,5,6,7,8,9];%����ڵ�
 %D=[2,4,3,5,6,5,7,6,8,9,8,9,9];%����ڵ� 
 %W=[2,2,2,2,2,1,2,1,2,2,2,2,0];%Ȩ��
% O=[1,1,2,2,2,3,3,3,3,4,4,4,5,5,5,6,6,6,7,7,7,7,8,8,8,9,9,10,10,10];
% D=[2,3,1,3,4,1,2,4,5,2,3,6,3,6,7,4,5,8,5,8,9,10,6,7,10,7,10,7,8,9];
% W=[5,7,10,2,5,7,2,3,5,5,3,8,5,3,3,8,3,2,3,6,5,7,2,6,5,5,10,7,5,10];
%O=[1 1 2 2 3 3 4 5 5 6 7 7 7 8 9 10];
%D=[2 3 3 4 4 5 6 6 7 8 8 9 10 10 10 10];
%W=[5 7 2 5 3 5 8 3 3 2 6 5 7 5 10 0];
%O=[1 1 2 2 2 2 3 3 3 4 4 4 4 5 5 6 6 6 7 7 7 8 8 8 9 9 9 10 10 10 ];
%D=[2 6 1 6 7 3 2 8 4 3 5 10 9 4 10 1 2 7 2 6 8 7 3 9 4 8 10 4 5 9];
%W=[7 5 7 2 3 5 5 3 3 3 5 7 6 5 10 5 3 5 3 5 8 8 3 2 6 2 5 7 10 5];

%SUE����
O=[1 1 4 4 5 5 6 6 7 7 8 9 9 10 11 11 12 12 13];
D=[5 12 5 9 6 9 7 10 8 11 2 10 13 11 2 3 6 8 3];
W=[12 12 12 24 12 12 12 12 12 12 12 12 24 12 12 12 12 36 12];
linkweight=sparse(O,D,W);%����ϡ�����

%num_nodes=length(O);
%����㵽�յ����·s

%����likelihood
theta=1.6;
%setp 2 ��ǰ����Ȩ��,��L��Ϊϡ���

%% �������
OD_pair=[4 1 2 3];
OD_demand=[1 1 4 4;2 3 2 3; 2500 2000 2000 2500];
ODnum=size(OD_demand,2);


T=sparse(O,D,W);
T=full(T);
T=T(:)';%��������ʱ��ͬ��ת����1*���ڵ���*�ڵ������Ĵ洢��ʽ
%��������
Maxcapacity=[3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000];
Maxcapacity=sparse(O,D,Maxcapacity);
Maxcapacity=Maxcapacity(:)';
G_vexnum=19;%����
G_nodenum=13;%�ڵ���
Total_link=G_nodenum*G_nodenum;
ZO=ones(1,length(O));
ZO1=sparse(O,D,ZO);%0-1����
G_graph=full(ZO1);

for i=1:ODnum
    origin=OD_demand(1,i);
    destination=OD_demand(2,i);
    pathset=findPath(G_graph,origin,destination,0);
    pathset=pathset(:,1:size(pathset,2)-1);
    %����0-1��������
    path_link_matrix{1,i} = construct2(pathset,G_nodenum);
    %����·��0-1���󣬷ֱ�洢��Ԫ����
    %P{i}=construct(pathset,G_nodenum,destination);
    %·������·��
%     [pathcost{i},path_link{i}]=convertpath(pathset,destination,ZO2,Maxcapacity,Free_time,G_vexnum);
%     free_time=sparse(O,D,Free_time);
%     free_time=full(free_time);
end
%% dial����
OD_input=[0 2500 2000 0;...
          0   0   0   0;...
          0   0   0   0;...
          0 2000 2500 0];

[count1 count2]=dial(theta,OD_pair,O,D,W,OD_input,linkweight);
%% 
X1=count1(:)';
k=1;error=0.3;
epsilon = 0.0001;%���ó�ʼ����������

while error>epsilon
    
%% step1������·����ʻʱ��time
    time=T.*(1+1.*(X1./Maxcapacity).^2);
    time(isnan(time))=0;
    
    for i=1:G_nodenum
        time(i+G_nodenum*(i-1))=0;
    end
    %���飨debug��
    
    for i=1:ODnum
        pathcost{i}=path_link_matrix{i}*time';
        pathcost{i}=pathcost{i}';
    end 
    pc2=pathcost{1};
 
    
    %% step2: ��step1�м������·��ʱ���OD��ͨ����ִ��һ��0-1���䣬�õ�һ�鸽�ӽ�ͨ��Y1
    for i = 1:ODnum
    [Min(i),short_path(i)]=min(pathcost{i}); 
    end
    Short=short_path
    Y1=zeros(1,Total_link);
%     for i = 1:G_nodenum
%     linkweight(:,i) = time(((i-1)*G_nodenum+1):i*G_nodenum)';
%     end
%     linkweight=sparse(linkweight);
%     
%     for i=1:length(O)
%         W(i)=linkweight(O(i),D(i));
% %     end 
%     [count1 count2]=dial(theta,OD_pair,O,D,W,OD_input,linkweight); 
%     Y1=count1(:)';
    for i = 1:ODnum
        temp = path_link_matrix{i};
        Y1 = temp(short_path(i),:).*OD_demand(3,i) + Y1;
    end
    panduan=Y1==X1;
    %% step3 �����·�ε�ǰ��ͨ��
    X2=X1+1./k*(Y1-X1); %   0< phi<1 phi=0.
    %% step4 �жϵ�·�������Ƿ�������������
    error= sum(abs(X1-X2))/sum(X1);
    %else ��������
    X1=X2;
    k=k+1;
end
fprintf('��������%d\n',k)

%% ��ԭ����
for i = 1:G_nodenum
    X(:,i) = X1(((i-1)*G_nodenum+1):i*G_nodenum)';
end


% count=count_positive+count_negative';
% plotHH(count)

