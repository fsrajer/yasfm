function score = computeScoreMyFirst(H)

e = eig(H);

if isreal(e)
    pairs = e([1 2; 1 3; 2 3]);
    pairs = bsxfun(@rdivide,pairs,pairs(:,1));
    score = min(abs(pairs(:,2) - pairs(:,1)));
else
    val = e(imag(e) > 0);
    score = abs(imag(val) / real(val));
end
end