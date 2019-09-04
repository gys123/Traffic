function Wi=weight2(r,Lsparse,Oi,Di,origin,O,D,destination)
%r=r(origin,:);
[a,b]=sort(r);%∂‘r≈≈–Ú
Wi=sparse(O,D,zeros(1,length(O)));
b=b(1:find(b==destination));
for i=b
    if i==origin
          Wi(i,Oi{i})=Lsparse(i,Oi{i});
    else 
          Wi(i,Oi{i})=Lsparse(i,Oi{i})*sum(Wi(Di{i},i));
    end
    
end
end 