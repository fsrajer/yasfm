function fs = fguiButtonShowNViewMatches(fs, fn)

if isempty(fs.ImgIdxs)
    idxs = 1:numel(fs.res{1}.cams);
else
    idxs = str2double(strsplit(fs.ImgIdxs));
end
  
matches = cell(1,numel(fs.res));
for i=1:numel(matches)
    matches{i} = fs.res{i}.nViewMatches;
end

showNViewMatches(fs.res{1}.cams,idxs,matches,...
        fs.subfigX,fs.subfigY);

end

