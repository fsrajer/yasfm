function [ims,nImgs] = readImgs(dir,imgFilenames,pairs)

nImgs=size(imgFilenames,2);

if exist('pairs','var')
    idxs=unique(pairs(:))';
else
    idxs=1:nImgs;
end
    
ims=cell(1,nImgs);
for idx=idxs
    %disp(['reading: ' num2str(idx)])
    ims{idx}=imread(fullfile(dir,imgFilenames{idx}));
end
end