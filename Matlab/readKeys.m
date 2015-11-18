function keys = readKeys(filename,containsOnlyLocation)

if ~exist('containsOnlyLocation','var')
    containsOnlyLocation=true;
end

fid = fopen(filename, 'r');

nKeys = fscanf(fid, '%d', 1);

if containsOnlyLocation
    keys = zeros(2, nKeys);
else
    keys = zeros(4, nKeys);
end

for i=1:nKeys
   keys(2,i) = fscanf(fid, '%lf', 1);
   keys(1,i) = fscanf(fid, '%lf', 1);
   if ~containsOnlyLocation
      keys(3,i) = fscanf(fid, '%lf', 1);
      keys(4,i) = fscanf(fid, '%lf', 1);
   end
end

fclose(fid);
end