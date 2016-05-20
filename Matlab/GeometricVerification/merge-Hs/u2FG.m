function [F,G] = u2FG(u1,u2)

H1 = findNormalization(u1);
H2 = findNormalization(u2);

u1 = H1*e2p(u1);
u2 = H2*e2p(u2);
M = zeros(8,9);
for i=1:8
    M(i,:) = [u2(1,i)*u1(1,i) u2(1,i)*u1(2,i) u2(1,i)*u1(3,i) ...
              u2(2,i)*u1(1,i) u2(2,i)*u1(2,i) u2(2,i)*u1(3,i) ...
              u2(3,i)*u1(1,i) u2(3,i)*u1(2,i) u2(3,i)*u1(3,i)];
end

[~,~,V] = svd(M);

g = V(:,end);
G = reshape(g,3,3)';

[U,S,V] = svd(G);
S(end,end) = 0;
F =U*S*V';

G = H2' * G * H1;
F = H2' * F * H1;

end

function H = findNormalization(u)

m=mean(u,2);
s=std(u,0,2);

H = [1/s(1) 0 0; 
     0 1/s(2) 0;
     0 0 1] * ...
     [1 0 -m(1);
      0 1 -m(2);
      0 0 1];

end