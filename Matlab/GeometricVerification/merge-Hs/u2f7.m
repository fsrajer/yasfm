function Fs = u2F7(u)

Z = [];
for j = 1:7
   Z(j,:) = reshape(u(1:3,j)*u(4:6,j)',1,9);
end

NullSp   = null(Z);
if size(NullSp,2) > 2 
   Fs = [];
   return; %degenerated sample
end

F1    = reshape(NullSp(:,1),3,3);
F2    = reshape(NullSp(:,2),3,3);
roots = slcm(F1,F2);

roots = roots(imag(roots) == 0);

for i = 1:length(roots)
   l  = roots(i);
   Ft = F1 * l + F2 * (1-l);
   Fs(:,:,i) = Ft;
end

%slcm		singular linear combination of matrices A and B
%	function la = slcm (A, B)
%	returns l = la[1:3], det(lA + (1-l)B) = 0 
function la = slcm (A, B)
A = A - B;
p(1) = det(A);
p(2) = det(Ki(1,A,B))+det(Ki(2,A,B))+det(Ki(3,A,B));
p(3) = det(Ki(1,B,A))+det(Ki(2,B,A))+det(Ki(3,B,A));
p(4) = det(B);
la = roots(p);

function A = Ki(i,A,B)
A(i,:) = B(i,:);

%seig			sorted eigenvalues
function [V,d] = seig(M)
[V,D]    = eig(M);
[d,s]    = sort(diag(D));
V        = V(:,s);
