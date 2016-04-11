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

function pairs = divideMatchesByGroups(pairsIn)
maxTrans = max(cellfun(@numel,{pairsIn.groups}));
if maxTrans == 0
    pairs = {pairsIn};
    return;
end
pairs = cell(maxTrans,1);
for iT=1:maxTrans
    pairs{iT} = repmat(struct('matches',[]),size(pairsIn));
    for i=1:size(pairsIn,1)
        for j=1:size(pairsIn,2)
            curr = pairsIn(i,j).groups;
            if numel(curr) >= iT
                sizes = cell2mat({curr.size});
                off = sum(sizes(1:(iT-1)));
                cnt = sizes(iT);
                pairs{iT}(i,j).matches = ...
                    pairsIn(i,j).matches(:,(off+1):(off+cnt));
            end
        end
    end
end
end

