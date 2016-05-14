function Ds = FDs(F,u)
%FDs	error trem for fundamentel m. - squares of Sampson's distances
%function Ds = FDs(F,u)
%where u are corespondences and F is fundamental matrix

rx1  = (F(:,1)' * u(1:3,:)).^2;
ry1  = (F(:,2)' * u(1:3,:)).^2;
rx2  = (F(1,:)  * u(4:6,:)).^2;
ry2  = (F(2,:)  * u(4:6,:)).^2;
r    = Fr(F,u);
Ds   = r ./ (rx1 + ry1 + rx2 + ry2);

function r = Fr(F,u)
%Fr	error trem for fundamentel m. - squares of algebraic distances
%function r = Fr(F,u)
%where u are corespondences and F is fundamental matrix

ep2        = F' * u(1:3,:);
r          = sum (ep2 .* u(4:6,:)) .^ 2;
