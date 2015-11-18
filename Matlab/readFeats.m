function feat = readFeats(filename)
% does not save descriptor

fid = fopen(filename, 'r');

nKeys = fscanf(fid, '%d', 1);
dim = fscanf(fid, '%d', 1);

feat = zeros(4, nKeys);

for i=1:nKeys
   feat(2,i) = fscanf(fid, '%lf', 1);
   feat(1,i) = fscanf(fid, '%lf', 1);
   scale = fscanf(fid, '%lf', 1);
   ori = fscanf(fid, '%lf', 1);
   if ~justPts
        feat(3,i) = scale;
        feat(4,i) = ori;
   end
   for skip=1:8
        fgets(fid, 1024);
   end
%    fscanf(fid, '%d', dim);
end

fclose(fid);
end