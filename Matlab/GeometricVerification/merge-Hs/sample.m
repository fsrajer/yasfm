%sample		takes randomly k numbers out of (1:n)
%	function sam = sample(k, n)
function sam = sample(k, n)
pom = randperm(n);
sam = pom(1:k);
