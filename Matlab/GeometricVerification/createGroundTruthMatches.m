%% setup
addpath ..

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
% pairsToLabel = [1:2:nImgs; 2:2:nImgs];
% 
% load(outFn);
% % labels = cell(nImgs,nImgs);

%% do labelling

waitfor(msgbox({'press buttons to label:','spacebar - dont know',...
    '0 - outlier','1,2,... - motion groups','escape - skip pair'}));

for pairIdx=1:size(pairsToLabel,2)
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
    if numel(labels{i,j}) ~= n
        labels{i,j} = -ones(1,n);
    end
    for k=1:n
        idx = order(k);
        m = matches(:,idx);
        key1 = data.cams(i).keys(m(1)).coord;
        key2 = data.cams(j).keys(m(2)).coord;
        key2(1) = key2(1) + offset;
        tit = ['id: ' num2str(k) '/' num2str(n)...
            ', score: ' num2str(dists(idx))];
        retVal = plotMatchAndAskForLabel(img,key1,key2,tit);
        if isinf(retVal)
            break;
        else
            labels{i,j}(idx) = retVal;
        end
        save(outFn,'labels');
    end
    save(outFn,'labels');
end

%% show labelled
figure;
for pairIdx=1:size(pairsToLabel,2)
    i = pairsToLabel(1,pairIdx);
    j = pairsToLabel(2,pairIdx);
    matches = data.pairs(i,j).matches;
    
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
    
    cams=data.cams;
    keys2 = cams(j).keys;
    for k=1:numel(keys2)
        keys2(k).coord(1) = keys2(k).coord(1)+offset;
    end
    
    gt = labels{i,j};
    for label=unique(gt)
        hold off;
        image(img);
        hold on;
        set(gca,'YDir','reverse');
        axis equal;
        axis([0 size(img,2) 0 size(img,1)]);
        
        plotMatches(cams(i).keys,keys2,matches(:,gt==label),0,[0 1 0],[0 1 0])
        title(['label: ' num2str(label)]);
        waitforbuttonpress;
        plotMatches(cams(i).keys,keys2,matches(:,gt==label),1,[0 1 0],[0 1 0])
        waitforbuttonpress;
    end
end
