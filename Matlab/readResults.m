function res = readResults(fn,ignoreFeats)

fid = fopen(fn,'r');

if fid == -1
    disp(['ERROR: Could not read: ' fn]);
    return;
end

res = [];
while ~feof(fid)
    line = fgetl(fid);
    if isempty(line) || line(1) == '#'
        continue
    else
        nFields = str2double(line);
        for iF = 1:nFields
            line = fgetl(fid);
            tokens = strsplit(line);
            name = tokens{1};
            if strcmp(name,'dir_')
                res.dir = fgetl(fid);
            elseif strcmp(name,'reconstructedCams_')
                fgetl(fid);
            elseif strcmp(name,'cams_')
                nCams = str2double(tokens{2});
                res.cams = readCams(fid,nCams,ignoreFeats);
            elseif strcmp(name,'queries_')
                nQueries = str2double(tokens{2});
                res.queries = readQueries(fid,nQueries);
            elseif strcmp(name,'nViewMatches_')
                nMatches = str2double(tokens{2});
                res.nViewMatches = readNViewMatches(fid,nMatches);
            elseif strcmp(name,'pts_')
                nPts = str2double(tokens{2});
                nFields = str2double(tokens{3});
                res.pts = readPts(fid,nPts,nFields);
            elseif strcmp(name,'points_')
                [res.nViewMatches,res.pts] = readPointsOld(fid);
            elseif strcmp(name,'pairs_')
                nPairs = str2double(tokens{2});
                nFields = str2double(tokens{3});
                res.pairs = readPairs(fid,nPairs,nFields,numel(res.cams));
            end
        end
    end
end
fclose(fid);
end

function cams = readCams(fid,nCams,ignoreFeats)
cams = {};
for iCam = 1:nCams
    line = fgetl(fid);
    tokens = strsplit(line);
    name = tokens{1};
    if strcmp(name,'StandardCamera')
        nLines = 12;
    elseif strcmp(name,'StandardCameraRadial')
        nLines = 15;
    else
       error(['ERROR: readCams: unknown camera class: ' name]); 
    end
    cams(iCam).fn = fgetl(fid);
    fgetl(fid);
    cams(iCam).featFn = fgetl(fid);
    for i=1:(nLines-3)
        fgetl(fid);
    end
    
    if ~ignoreFeats
        cams(iCam).keys = readKeys(cams(iCam).featFn);
    end
end
end

function queries = readQueries(fid,nQueries)
queries = cell(nQueries,1);
for i=1:nQueries
    n = fscanf(fid,'%i',1);
    queries{i} = fscanf(fid,'%i',n);
end
fgetl(fid);
end

function pts = readPts(fid,nPts,nFields)
pts = repmat(struct('coord',[]),nPts,1);
fields = cell(nFields,1);
for i=1:nFields
    fields{i} = fgetl(fid);
end
for i=1:nPts
    line = fgetl(fid);
    tokens = str2double(strsplit(line));
    start = 1;
    if nFields >= 1 && strcmp(fields{1},'coord')
        pts(i).coord = tokens(start:(start+2));
        start = start + 3;
    end
    if nFields >= 1 && strcmp(fields{2},'views')
        pts(i).views = retrieveNViewMatch(tokens(start:end));
        start = start + 1 + 2*size(pts(i).views,1);
    end
    if nFields >= 2 && strcmp(fields{3},'viewsToAdd')
        pts(i).viewsToAdd = retrieveNViewMatch(tokens(start:end));
        start = start + 1 + 2*size(pts(i).viewsToAdd,1);
    end
    if nFields >= 3 && strcmp(fields{4},'color')
        pts(i).color = tokens(start:(start+2));
        start = start + 3;
    end
end
end

function matches = readNViewMatches(fid,nMatches)
matches = cell(nMatches,1);
for i=1:nMatches
    line = fgetl(fid);
    tokens = str2double(strsplit(line));
    matches{i} = retrieveNViewMatch(tokens);
end
end

function [matchesToDo,pts] = readPointsOld(fid)
line = fgetl(fid);
nFields = str2double(line);
matchesToDo = [];
pts = [];
for iF=1:nFields
    line = fgetl(fid);
    tokens = strsplit(line);
    name = tokens{1};
    if strcmp(name,'matchesToReconstruct_')
        n = str2double(tokens{2});
        matchesToDo = cell(n,1);
        for i=1:n
            line = fgetl(fid);
            tokens = str2double(strsplit(line));
            matchesToDo{i} = retrieveNViewMatch(tokens);
        end
    elseif strcmp(name,'ptCoord_')
        n = str2double(tokens{2});
        coord = fscanf(fid,'%lf',[3,n]);
        fgetl(fid);
        if isempty(pts)
            pts = repmat(struct('coord',[0 0 0]'),n,1);
        end
        for i=1:n
           pts(i).coord = coord(:,i); 
        end
    elseif strcmp(name,'ptData_')
        n = str2double(tokens{2});
        nFields_ = str2double(tokens{3});
        if isempty(pts)
            pts = repmat(struct('reconstructed',[]),n,1);
        end
        fields_ = cell(nFields_,1);
        for i=1:nFields_
            fields_{i} = fgetl(fid);
        end
        for i=1:n
            line = fgetl(fid);
            tokens = str2double(strsplit(line));
            start = 1;
            if nFields_ >= 1 && strcmp(fields_{1},'reconstructed')
                pts(i).reconstructed = ...
                    retrieveNViewMatch(tokens(start:end));
                start = start + 1 + 2*size(pts(i).reconstructed,1);
            end
            if nFields_ >= 2 && strcmp(fields_{2},'toReconstruct')
                pts(i).toReconstruct = ...
                    retrieveNViewMatch(tokens(start:end));
                start = start + 1 + 2*size(pts(i).toReconstruct,1);
            end
            if nFields_ >= 3 && strcmp(fields_{3},'color')
                pts(i).color = tokens(start:(start+2));
                start = start + 3;
            end
        end
    end
end
end

function match = retrieveNViewMatch(data)
m = data(1);
match = repmat(struct('img',-1,'key',-1),m,1);
for j=0:(m-1)
    match(j+1).img = data(j*2+2) + 1; % adding 1 because of matlab indexing
    match(j+1).key = data(j*2+3) + 1; % adding 1 because of matlab indexing
end
end

function pairs = readPairs(fid,nPairs,nFields,nCams)
fields = cell(nFields,1);
for i=1:nFields
    fields{i} = fgetl(fid);
end
pairs = repmat(struct('matches',[]),nCams,nCams);
for i=1:nPairs
    line = fgetl(fid);
    tokens = str2double(strsplit(line));
    im1 = tokens(1) + 1; % adding 1 because of matlab indexing
    im2 = tokens(2) + 1; % adding 1 because of matlab indexing
    if nFields >= 1 && strcmp(fields{1},'matches')
        nMatches = fscanf(fid,'%i',1);
        pairs(im1,im2).matches = ...
            fscanf(fid,'%i',[2 nMatches]) + 1; % adding 1 because of matlab indexing
        fgetl(fid);
    end
    if nFields >= 2 && strcmp(fields{2},'dists')
        nDists = fscanf(fid,'%i',1);
        pairs(im1,im2).dists = fscanf(fid,'%lf',[1 nDists]);
        fgetl(fid);
    end
    if nFields >= 3 && strcmp(fields{3},'supportSizes')
        nSuppSz = fscanf(fid,'%i',1);
        fgetl(fid);
        pairs(im1,im2).supportSizes = fscanf(fid,'%i',[1 nSuppSz]);
        fgetl(fid);
    end
end
end