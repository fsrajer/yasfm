function score = computeFundMatErr(H1,H2,u11,u12,u21,u22,u31,u32)

% H = inv(H2)*H1;
% [V,D] = eig(H);
% d = diag(D);
% 
% if isreal(d)
%     tripletsIdxs = [1 2 3; 1 3 2; 2 3 1];
%     triplets = d(tripletsIdxs);
%     triplets = bsxfun(@rdivide,triplets,triplets(:,1));
%     diff = abs(triplets(:,2) - triplets(:,1));
%     [~,idx] = min(diff);
%     idx = tripletsIdxs(idx,3);
% else
%     idx = find(imag(d)==0);
% end
% 
% e1 = V(:,idx);
% e2 = H1*e1;
% F1 = [0 -e2(3) e2(2);e2(3) 0 -e2(1);-e2(2) e2(1) 0]*H1;
% F2 = [0 -e2(3) e2(2);e2(3) 0 -e2(1);-e2(2) e2(1) 0]*H2;
 
% err1 = fds(F1,[e2p(u11) e2p(u21);e2p(u12) e2p(u22)]);
% err2 = fds(F2,[e2p(u11) e2p(u21);e2p(u12) e2p(u22)]);
% score = mean([err1 err2]);

[Fbest,inl]=ransac_f([e2p(u11) e2p(u21);e2p(u12) e2p(u22)],sqrt(5.),0.95);

% err = fds(Fbest,[e2p(u11) e2p(u21);e2p(u12) e2p(u22)]);
% score2 = sum(err < 20.);

err = fds(Fbest,[e2p(u31);e2p(u32)]);
% score2 = sum(err(err < 20.));
score = sum(err < 20.);
% score = mean(err);

end