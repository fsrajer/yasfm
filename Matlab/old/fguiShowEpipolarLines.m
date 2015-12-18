function fguiShowEpipolarLines(fs, fn)

pairsToShow = parseInitPairsToShow(fs.ImgIdxs,fs.ImgPairs);

showEpipolarLines_(fullfile(fs.DataDirectory,fs.DatasetSubdirectory), ...
            fs.ListImgs, fs.ListKeys, ...
            fs.MatchesEG,fs.Transforms,fs.MaxEpLinesInImage,...
            pairsToShow,fs.subfigX,fs.subfigY);

end

function showEpipolarLines_(dir, listImgsFilename, listKeysFilename, ...
                matchesFilename,trabsformsFilename,maxLinesInImage,...
                pairsToShow,subfigX,subfigY)

if ~exist('subfigX','var')
    subfigX = 4;
end
if ~exist('subfigY','var')
    subfigY = 3;
end

subfigX = floor(subfigX/2);

listImgs = readFilenames(fullfile(dir,listImgsFilename));
listKeys = readFilenames(fullfile(dir,listKeysFilename));

nImgs = size(listImgs,2);
if(nImgs ~= size(listKeys,2))
    disp('number of images and number of keypoint files do not match');
    return;
end

if ~exist('pairsToShow','var') || isempty(pairsToShow)
    pairsToShow = getAllPairs(nImgs);
elseif size(pairsToShow,1)==1
    idxs=pairsToShow;
    pairsToShow = getAllPairs(nImgs);
    toKeep = ismember(pairsToShow(1,:),idxs) | ismember(pairsToShow(2,:),idxs);
    pairsToShow=pairsToShow(:,toKeep);
end

imgs = readImgs(dir,listImgs,pairsToShow);
keys = readKeysAll(dir,listKeys,pairsToShow);

matches = readMatches(fullfile(dir,matchesFilename),nImgs);
Fs = readTransforms(fullfile(dir,trabsformsFilename),nImgs);

subfigIdx = 1;
for pair=pairsToShow
    i=pair(1);
    j=pair(2);
    
    if isempty(matches{i,j})
       continue;
    end
    
    subfig(subfigY, subfigX, mod(subfigIdx-1,subfigX*subfigY) + 1);
    subfigIdx = subfigIdx+1;
    hold on;
    
    nth = ceil(size(matches{i,j},2)/maxLinesInImage);
    drawEpipolarLines(imgs{i},imgs{j},...
        keys{i}(1:2,matches{i,j}(1,:)), keys{j}(1:2,matches{i,j}(2,:)),...
        Fs{i,j},nth);
    
    tit = ['pair: ' num2str(i) '-' num2str(j)];
    title(tit);
    
    hold off
end

end