function [F, G, errsim1, errsim2] = u2FG_optim(u1,u2,IX)

n = size(IX,2);
tuples = nchoosek(1:n,8);

F=[];
G=[];
minMaxErr=Inf;
errsim1=[];
errsim2=[];
for i=1:size(tuples,1)
    idxs = tuples(i,:);
    [F_,G_] = u2FG(u1(:,idxs),u2(:,idxs));
    if isempty(F_)
        continue;
    end
    [errs1_,errs2_] = computeSqErr(F_,u1,u2);
    maxErr_ = max(errs1_+ errs2_);
    if minMaxErr>maxErr_
        minMaxErr = maxErr_;
        F = F_;
        G = G_;
        errsim1=errs1_;
        errsim2=errs2_;
    end
end

errsim1 = sqrt(errsim1);
errsim2 = sqrt(errsim2);

end

function [errs1,errs2] = computeSqErr(F,u1,u2)

l1 = F'*e2p(u2);
l2 = F*e2p(u1);

l1=normLine(l1);
l2=normLine(l2);

errs1 = (diag(l1'*e2p(u1))').^2;
errs2 = (diag(l2'*e2p(u2))').^2;

end

function ln = normLine(l)

factor = sqrt(l(1,:).^2 + l(2,:).^2);
ln = bsxfun(@rdivide,l,factor);

end

