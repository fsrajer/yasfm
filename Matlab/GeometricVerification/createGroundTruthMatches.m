%% setup
addpath ..
% pairsToLabel = [29 30; 59 60; 1 2; 51 52; 61 62; 55 56; 57 58; 53 54]';
pairsToLabel = [29 30; 61 62; 1 2; 45 46; 47 48; 51 52; 55 56; 59 60]';

dataDir = 'C:\Users\Filip\Dropbox\pairs';
inFn = 'tentatively_matched_all.txt';
outFn = [inFn(1:end-4) '_ground_truth.mat'];

inFn = fullfile(dataDir,inFn);
outFn = fullfile(dataDir,outFn);
separatorWidthScale = 0.03125;

%% read results
% ignoreFeats = false;
% data = readResults(inFn,ignoreFeats);
% nImgs = numel(data.cams);
% 
% % load(outFn);
% labels = cell(nImgs,nImgs);

%% do labelling

waitfor(msgbox({'press buttons to label:','spacebar - dont know',...
    '0 - outlier','1,2,... - motion groups'}));

for pairIdx=6:size(pairsToLabel,2)
    i = pairsToLabel(1,pairIdx);
    j = pairsToLabel(2,pairIdx);
    matches = data.pairs(i,j).matches;
    dists = data.pairs(i,j).dists;
    n = numel(dists);
    
    im1 = imread(data.cams(i).fn);
    im2 = imread(data.cams(j).fn);
    
    w1 = size(im1,2);
    h1 = size(im1,1);
    w2 = size(im2,2);
    h2 = size(im2,1);
    offset = w1+ceil(separatorWidthScale*w1);
    
    img = uint8(zeros(max(h1,h2),offset+w2,size(im1,3)));
    img(1:h1,1:w1,:) = im1;
    img(1:h2,(offset+1):(offset+w2),:) = im2;
    
    [~,order] = sort(dists);
    labels{i,j} = -ones(1,n);
    for k=1:n
        idx = order(k);
        m = matches(:,idx);
        key1 = data.cams(i).keys(m(1)).coord;
        key2 = data.cams(j).keys(m(2)).coord;
        key2(1) = key2(1) + offset;
        tit = ['id: ' num2str(k) '/' num2str(n)...
            ', score: ' num2str(dists(idx))];
        labels{i,j}(idx) = plotMatchAndAskForLabel(img,...
            key1,key2,tit);
        save(outFn,'labels');
    end
    save(outFn,'labels');
end