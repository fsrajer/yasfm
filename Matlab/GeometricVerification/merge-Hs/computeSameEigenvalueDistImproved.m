function score = computeSameEigenvalueDistImproved(H)

e = eig(H);

if isreal(e)
    triplets = e([1 2 3; 1 3 2; 2 3 1]);
    triplets = bsxfun(@rdivide,triplets,0.5*sum(triplets(:,1:2),2));
    diff = sum(abs(1-triplets(:,1:2)),2);
    score = min(diff(triplets(:,3)>0));
    if isempty(score)
        score = Inf;
    end
else
    val = e(imag(e) > 0);
    score = 2*abs(imag(val) / real(val));
    if sign(e(imag(e) == 0)) ~= sign(real(val))
        score = Inf;
    end
end

end