function fs = fguiButtonShowKeypoints(fs, fn)

if isempty(fs.ImgIdxs)
    idxs = 1:numel(fs.res.cams);
else
    idxs = str2double(strsplit(fs.ImgIdxs));
end
    
showKeypoints(fs.res, fs.SimplePoints, idxs, fs.subfigX, fs.subfigY); 

end