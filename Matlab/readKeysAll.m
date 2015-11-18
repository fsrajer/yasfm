function keys = readKeysAll(dir,keysFilenames,pairs)

nImgs=size(keysFilenames,2);

if exist('pairs','var')
    idxs=unique(pairs(:))';
else
    idxs=1:nImgs;
end
    
keys=cell(1,nImgs);
for idx=idxs
    %disp(['reading: ' num2str(idx)])
    keys{idx}=readKeys(fullfile(dir,keysFilenames{idx}));
end
end