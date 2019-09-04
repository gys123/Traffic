function Lsparse=likelihood(r,s,theta,W,O,D)

L=zeros(1,length(O));

for k=1:length(O)
    if r(O(k))<r(D(k))&s(O(k))>s(D(k))
        L(k)=exp(theta*(r(D(k))-r(O(k))-W(k)));
    end
end
Lsparse=sparse(O,D,L);
end
