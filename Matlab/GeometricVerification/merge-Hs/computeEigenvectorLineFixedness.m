function score = computeEigenvectorLineFixedness(H)


[V,D] = eig(inv(H'));

l0 = rand(3,1);
score = [];
for ie = find(imag(diag(D))==0)'
    l=V(:,ie);
    
    alpha=0:0.5:(pi);
    x0 = cross(l,l0);
    x0 = x0/norm(x0);
    x=zeros(3,numel(alpha));
    for i=1:numel(alpha)
        R = vrrotvec2mat([l;alpha(i)]);
        x(:,i) = R * x0;
    end
    y=H*x;
    err=zeros(size(alpha));
    for i=1:numel(alpha)
        err(i) = dot(x(:,i),y(:,i))/(norm(y(:,i)));
    end
    score = max([min(abs(err)) score]);
end
end