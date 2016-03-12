function matches = splitMatchesToGroups(pair)

if ~isfield(pair,'supportSizes') || isempty(pair.supportSizes)
    matches = {pair.matches};
    return;
end

ss = pair.supportSizes;
matches = cell(1,numel(ss));
off = 0;
for i=1:numel(ss)
    sz = ss(i);
    matches{i} = pair.matches(:,(1:sz)+off);
    off = off + sz;
end
end