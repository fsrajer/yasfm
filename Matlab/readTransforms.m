function Fs = readTransforms(filename,nImgs)

if exist('nImgs','var')
    Fs = cell(nImgs,nImgs);
else
    Fs = {};
end

fid = fopen(filename,'r');

nPairs = fscanf(fid, '%d',1);
for i=1:nPairs
    img1 = fscanf(fid, '%d',1) + 1;
    img2 = fscanf(fid, '%d',1) + 1;
    Fs{img1,img2} = reshape(fscanf(fid,'%f',9),[3 3])';
end

fclose(fid);

end