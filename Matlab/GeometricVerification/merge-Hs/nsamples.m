%SampleCnt calculates number of samples needed to be done

function SampleCnt = nsamples(I, N, m, p)

if p > 1
  error('Conf must be less then or equal to 1');
end

q  = prod ([(I-m+1) : I] ./ [(N-m+1) : N]);

if q < eps
   SampleCnt = Inf;
else  
   SampleCnt  = log(1 - p) / log(1 - q);
end

if SampleCnt < 1
   SampleCnt = 1;
end
