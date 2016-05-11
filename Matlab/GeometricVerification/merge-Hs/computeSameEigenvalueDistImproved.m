function score = computeSameEigenvalueDistImproved(H)

e = eig(H);

if isreal(e)
    triplets = e([1 2 3; 1 3 2; 2 3 1]);
    triplets = bsxfun(@rdivide,triplets,triplets(:,1));
    diff = abs(triplets(:,2) - triplets(:,1));
    score = min(diff(triplets(:,3)>0));
    if isempty(score)
        score = Inf;
    end
else
    val = e(imag(e) > 0);
    score = abs(imag(val) / real(val));
    if sign(e(imag(e) == 0)) ~= sign(real(val))
        score = Inf;
    end
end

end