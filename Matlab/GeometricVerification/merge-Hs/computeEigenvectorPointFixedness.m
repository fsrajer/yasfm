function score = computeEigenvectorPointFixedness(H)


[V,D] = eig(H);

x0 = rand(3,1);
score = [];
for ie = find(imag(diag(D))==0)'
    x=V(:,ie);
    
    alpha=0:0.5:(pi);
    l0 = cross(x,x0);
    l0 = l0/norm(l0);
    l=zeros(3,numel(alpha));
    for i=1:numel(alpha)
        R = vrrotvec2mat([x;alpha(i)]);
        l(:,i) = R * l0;
    end
    p=inv(H')*l;
    err=zeros(size(alpha));
    for i=1:numel(alpha)
        err(i) = dot(l(:,i),p(:,i))/(norm(p(:,i)));
    end
    score = max([min(abs(err)) score]);
end
end