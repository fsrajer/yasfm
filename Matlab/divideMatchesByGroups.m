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
                pairs{iT}(i,j).groups = ...
                    pairsIn(i,j).groups(iT);
            end
        end
    end
end
end