function  path_link_matrix=construct2(pathset,G_nodenum)
 pathnum = size(pathset,1);
 path_link_matrix = zeros(pathnum,G_nodenum*G_nodenum);
for i = 1:pathnum 
    pathtemp = zeros(G_nodenum);
    for j = 1:size(pathset,2)-1
        if pathset(i,j+1) == 0
            break;
        else
            pathtemp(pathset(i,j),pathset(i,j+1))=1;
        end
    end
    path_link_matrix(i,:) = pathtemp(:);
end
end