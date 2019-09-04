function Xi=flow2(s,O,D,Oi,Di,Wi,destination,Q,origin)
%r=r(origin,:);
[a,b]=sort(s);
Xi=sparse(O,D,zeros(1,length(O)));
b=b(1:find(b==origin));
 for i=b
        %if Oi{i}==9 
        if i==destination
        %if Oi{i}==destination 
            Xi(Di{i},i)=Q*Wi(Di{i},i)/sum(Wi(Di{i},i));
            %Xi(Di{i},i)=Wi(Di{i},i)*sum(Xi(i,Oi{i}))/sum(Wi(Di{i},i));
        %end
        else 
        %if  Wi(Di{i},i)~=0
        %else
            Xi(Di{i},i)=Wi(Di{i},i)*sum(Xi(i,Oi{i}))/sum(Wi(Di{i},i));
        %end
        end
    %end
 end
Xi(isnan(Xi))=0;     
end