function pairs = getAllPairs(nImgs)
nPairs = floor((nImgs*(nImgs-1))/2);
pairs=zeros(2,nPairs);
count=1;
for i=1:nImgs
    for j=i+1:nImgs
        pairs(:,count) = [i;j];
        count = count+1;
    end
end
end