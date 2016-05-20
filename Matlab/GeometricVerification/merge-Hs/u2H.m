function H = u2H(u,u0)

M = [e2p(u)' zeros(4,3) -repmat(u0(1,:)',1,3).*e2p(u)';
     zeros(4,3) e2p(u)' -repmat(u0(2,:)',1,3).*e2p(u)'];

[~,S,V] = svd(M);

if abs(S(8,8)) < 1e-10
    H=[];
else
    h = V(:,end);
    H = reshape(h,3,3)';
    
    if abs(h(end))>1e-8
        H=H/h(end);
    end
end

end