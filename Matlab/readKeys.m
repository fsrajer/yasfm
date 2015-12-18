function keys = readKeys(filename)

fid = fopen(filename, 'r');

if fid == -1
    error(['ERROR: readKeys: cannot read ' filename]);
end

nKeys = fscanf(fid, '%i', 1);
dim = fscanf(fid, '%i', 1);

keys = repmat(struct('coord',[0;0],'scale',0,'orientation',0),nKeys,1);
for i=1:nKeys
    keys(i).coord(2) = fscanf(fid, '%lf', 1);
    keys(i).coord(1) = fscanf(fid, '%lf', 1);
    keys(i).scale = fscanf(fid, '%lf', 1);
    keys(i).orientation = fscanf(fid, '%lf', 1);
    nLines = ceil(dim / 20) + 1;
    for j=1:nLines
        fgetl(fid);
    end
end

fclose(fid);
end