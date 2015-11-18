function fguiShowKeypoints(fs, fn)

idxsStr = strsplit(fs.ImgIdxs);
idxs=zeros(1,size(idxsStr,2));
for i=1:size(idxsStr,2)
    idxs(i) = str2double(idxsStr{i});
end

simplePoints = true;

showKeypoints(fullfile(fs.DataDirectory,fs.DatasetSubdirectory), fs.ListImgs,...
    fs.ListKeys, simplePoints, idxs, fs.subfigX, fs.subfigY); 

end