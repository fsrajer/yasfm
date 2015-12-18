function fs = fguiButtonShowMatches(fs, fn)

pairsToShow = parseInitPairsToShow(fs.ImgIdxs,fs.ImgPairs,...
    numel(fs.res.cams));

showMatches(fs.res,fs.ShowAllKeys,fs.ShowLines,pairsToShow,...
            fs.subfigX,fs.subfigY);
end

function pairs = parseInitPairsToShow(idxsStr,pairsIdxsStr,nCams)

if ~isempty(pairsIdxsStr)
    pairsIdxsStr = strsplit(pairsIdxsStr);
    pairs = zeros(2,size(pairsIdxsStr,2));
    for i=1:size(pairsIdxsStr,2)
        s = find(pairsIdxsStr{i}=='-');
        idx1 = str2double(pairsIdxsStr{i}(1:s-1));
        idx2 = str2double(pairsIdxsStr{i}(s+1:end));
        pairs(:,i) = [idx1;idx2];
    end
else
    idxs = str2double(strsplit(idxsStr));
    pairs = [];
    for i=idxs
        for j=1:nCams
            if i<j
                pairs = [pairs [i;j]];
            elseif i>j
                pairs = [pairs [j;i]];
            end
        end
    end
    pairs = unique(pairs','rows')';
end

end

