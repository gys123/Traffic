% Guyangsong 07/13/2018  
% 本方法需要用到matlab网络图工具箱
%% 输入初始信息
 %O=[1,1,2,2,3,4,4,5,5,6,7,8,9];%网络节点
 %D=[2,4,3,5,6,5,7,6,8,9,8,9,9];%网络节点 
 %W=[2,2,2,2,2,1,2,1,2,2,2,2,0];%权重
% O=[1,1,2,2,2,3,3,3,3,4,4,4,5,5,5,6,6,6,7,7,7,7,8,8,8,9,9,10,10,10];
% D=[2,3,1,3,4,1,2,4,5,2,3,6,3,6,7,4,5,8,5,8,9,10,6,7,10,7,10,7,8,9];
% W=[5,7,10,2,5,7,2,3,5,5,3,8,5,3,3,8,3,2,3,6,5,7,2,6,5,5,10,7,5,10];
%O=[1 1 2 2 3 3 4 5 5 6 7 7 7 8 9 10];
%D=[2 3 3 4 4 5 6 6 7 8 8 9 10 10 10 10];
%W=[5 7 2 5 3 5 8 3 3 2 6 5 7 5 10 0];
%O=[1 1 2 2 2 2 3 3 3 4 4 4 4 5 5 6 6 6 7 7 7 8 8 8 9 9 9 10 10 10 ];
%D=[2 6 1 6 7 3 2 8 4 3 5 10 9 4 10 1 2 7 2 6 8 7 3 9 4 8 10 4 5 9];
%W=[7 5 7 2 3 5 5 3 3 3 5 7 6 5 10 5 3 5 3 5 8 8 3 2 6 2 5 7 10 5];

%SUE输入
O=[1 1 4 4 5 5 6 6 7 7 8 9 9 10 11 11 12 12 13];
D=[5 12 5 9 6 9 7 10 8 11 2 10 13 11 2 3 6 8 3];
W=[12 12 12 24 12 12 12 12 12 12 12 12 24 12 12 12 12 36 12];
linkweight=sparse(O,D,W);%构建稀疏矩阵

%num_nodes=length(O);
%求起点到终点最短路s

%计算likelihood
theta=1.6;
%setp 2 先前计算权重,将L变为稀疏矩

%% 网络参数
OD_pair=[4 1 2 3];
OD_demand=[1 1 4 4;2 3 2 3; 2500 2000 2000 2500];
ODnum=size(OD_demand,2);


T=sparse(O,D,W);
T=full(T);
T=T(:)';%将自由流时间同样转换成1*（节点数*节点数）的存储形式
%单向网络
Maxcapacity=[3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000 3000];
Maxcapacity=sparse(O,D,Maxcapacity);
Maxcapacity=Maxcapacity(:)';
G_vexnum=19;%边数
G_nodenum=13;%节点数
Total_link=G_nodenum*G_nodenum;
ZO=ones(1,length(O));
ZO1=sparse(O,D,ZO);%0-1矩阵
G_graph=full(ZO1);

for i=1:ODnum
    origin=OD_demand(1,i);
    destination=OD_demand(2,i);
    pathset=findPath(G_graph,origin,destination,0);
    pathset=pathset(:,1:size(pathset,2)-1);
    %构造0-1关联矩阵
    path_link_matrix{1,i} = construct2(pathset,G_nodenum);
    %构造路段0-1矩阵，分别存储在元胞里
    %P{i}=construct(pathset,G_nodenum,destination);
    %路段延误，路径
%     [pathcost{i},path_link{i}]=convertpath(pathset,destination,ZO2,Maxcapacity,Free_time,G_vexnum);
%     free_time=sparse(O,D,Free_time);
%     free_time=full(free_time);
end
%% dial分配
OD_input=[0 2500 2000 0;...
          0   0   0   0;...
          0   0   0   0;...
          0 2000 2500 0];

[count1 count2]=dial(theta,OD_pair,O,D,W,OD_input,linkweight);
%% 
X1=count1(:)';
k=1;error=0.3;
epsilon = 0.0001;%设置初始误差及迭代次数

while error>epsilon
    
%% step1：更新路段行驶时间time
    time=T.*(1+1.*(X1./Maxcapacity).^2);
    time(isnan(time))=0;
    
    for i=1:G_nodenum
        time(i+G_nodenum*(i-1))=0;
    end
    %检验（debug）
    
    for i=1:ODnum
        pathcost{i}=path_link_matrix{i}*time';
        pathcost{i}=pathcost{i}';
    end 
    pc2=pathcost{1};
 
    
    %% step2: 按step1中计算出的路段时间和OD交通量，执行一次0-1分配，得到一组附加交通量Y1
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
    %% step3 计算各路段当前交通量
    X2=X1+1./k*(Y1-X1); %   0< phi<1 phi=0.
    %% step4 判断当路段流量是否满足收敛条件
    error= sum(abs(X1-X2))/sum(X1);
    %else 继续迭代
    X1=X2;
    k=k+1;
end
fprintf('迭代次数%d\n',k)

%% 还原矩阵
for i = 1:G_nodenum
    X(:,i) = X1(((i-1)*G_nodenum+1):i*G_nodenum)';
end


% count=count_positive+count_negative';
% plotHH(count)

