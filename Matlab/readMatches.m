function matches = readMatches(matchesFile, nImgs)
% please note that indices of both images and keys
% are increased by one due to matlab indexing

if exist('nImgs','var')
    matches = cell(nImgs, nImgs);
else
    matches = {};
end

fid = fopen(matchesFile,'r');
nPairs = fscanf(fid, '%d',1);
for i=1:nPairs
    img1 = fscanf(fid, '%d',1) + 1;
    img2 = fscanf(fid, '%d',1) + 1;
    nMatches = fscanf(fid, '%d',1);
    
    matches{img1,img2} = zeros(2,nMatches);
    for j=1:nMatches
        matches{img1,img2}(:,j) = fscanf(fid, '%d', 2) + 1;
        fscanf(fid, '%lf', 1); %dist not interesting for now
    end
end

fclose(fid);

end