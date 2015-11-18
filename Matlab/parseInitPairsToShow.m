function pairs = parseInitPairsToShow(idxsStr, pairsIdxsStr)

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
    idxsStr = strsplit(idxsStr);
    pairs=zeros(1,size(idxsStr,2));
    for i=1:size(idxsStr,2)
        pairs(i) = str2double(idxsStr{i});
    end
end

end