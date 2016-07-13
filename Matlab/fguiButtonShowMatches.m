function fs = fguiButtonShowMatches(fs, fn)

pairsToShow = parseInitPairsToShow(fs.ImgIdxs,fs.ImgPairs,...
    numel(fs.res{1}.cams));

pairs = {};
for i=1:numel(fs.res)
    if ~isfield(fs.res{i}.pairs,'groups')
        pairs{end+1} = fs.res{i}.pairs;
    else
        tmp = divideMatchesByGroups(fs.res{i}.pairs);
        pairs((end+1):(end+numel(tmp))) = tmp;
    end
end

showMatches(fs.res{1}.cams,pairs,fs.ShowAllKeys,fs.ShowLines,pairsToShow,...
            fs.subfigX,fs.subfigY);
end

