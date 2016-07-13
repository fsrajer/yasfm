function matches = splitMatchesToGroups(pair)

if ~isfield(pair,'groups') || isempty(pair.groups)
    matches = {pair.matches};
    return;
end

groups = pair.groups;
matches = cell(1,numel(groups));
off = 0;
for i=1:numel(groups)
    sz = groups(i).size;
    matches{i} = pair.matches(:,(1:sz)+off);
    off = off + sz;
end
end