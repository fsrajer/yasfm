%% setup
addpath ..

% to be set by the user
dataDir = 'C:\Users\Filip\Dropbox\pairs\0.8';
allFn = 'tentatively_matched_all.txt';
gtFn = [allFn(1:end-4) '_ground_truth.mat'];
methods={};
methods{end+1} = 'ratio';
methods{end+1} = 'ratio-unique';
% methods{end+1} = 'seq-loprosac';
% methods{end+1} = 'seq-loprosac-opt';
methods{end+1} = 'H-EG-greedy';
% methods{end+1} = 'H-EG-greedy-';
methods{end+1} = 'gv';
% methods{end+1} = 'gv-fast';

colsMap = containers.Map;
colsMap('ratio') = [230 159 0]/255; % orange
colsMap('ratio-unique') = [213 94 0]/255; % red-ish (vermilion)
colsMap('gv') = [0 114 178]/255; % blue
colsMap('H-EG-greedy') = [0 0 0]; % black
% colsMap('') = [220 208 66]/255; % dark yellow
colsMap('seq-loprosac-opt') = [204 121 167]/255; % pink
% colsMap('') = [86 180 233]/255; % sky blue
colsMap('H-EG-greedy-') = [0 158 115]/255; % bluish green
colsMap('gv-fast') = [0 158 115]/255; % bluish green
colsMap('seq-loprosac') = [.8 0 .8]; % magenta
% colsMap('') = [0.6 0.5 0.1]; % brown
% colsMap('') = [0 .8 .8]; % cyan

allFn = fullfile(dataDir,allFn);
gtFn = fullfile(dataDir,gtFn);
nMethods = numel(methods);

%% list all result files
fns = cell(1,nMethods);
matlabDir = pwd;
for i=1:nMethods
    d = fullfile(dataDir,methods{i});
    cd(d);
    fns{i} = cellstr(ls('matched*'));
    
    % sort based on numbers
    r = regexp(fns{i},'\d+','match'); %find numbers
    [~, ind] = sort(cellfun(@(c) str2double(c{end}), r)); % convert to numbers and sort
    fns{i} = fns{i}(ind); %// use index to build result
end
cd(matlabDir);

%% read data
disp(['reading: features and gt']);
ignoreFeats = false;
data = readResults(allFn,ignoreFeats);
allPairs = data.pairs;
nImgs = numel(data.cams);

ld = load(gtFn);
labels = ld.labels;

ignoreFeats = true;
estPairs = cell(1,nMethods);
for i=1:nMethods
    disp(['reading: ' methods{i}]);
    estPairs{i} = cell(1,numel(fns{i}));
    for j=1:numel(fns{i})
        tmp = readResults(fullfile(dataDir,methods{i},fns{i}{j}),ignoreFeats);
        estPairs{i}{j} = tmp.pairs;
    end
end

%% compute precision and recall
prNoGroups=cell(1,nMethods);
rcNoGroups=cell(1,nMethods);
prGroups=cell(1,nMethods);
rcGroups=cell(1,nMethods);
pairsIdxs=[];
for i=1:nImgs
    for j=(i+1):nImgs
        gt = labels{i,j};
        
        toUse = (gt ~= -1);
        n = sum(toUse);
        gt(~toUse) = [];
        
        if n>0
            pairsIdxs(:,end+1) = [i;j];
            all = allPairs(i,j).matches(:,toUse);
            
            gtCurr = gt;
            gtCurr(gtCurr>0) = 1; % unite groups
  
            for im=1:nMethods
                nFns = numel(fns{im});
                pr = zeros(1,nFns);
                rc = zeros(1,nFns);
                for k=1:nFns
                    curr = estPairs{im}{k}(i,j).matches;
                    est = ismember(all',curr','rows')';
                    [pr(k),rc(k)] = computePrecisionRecall(gtCurr,est);
                end
                prNoGroups{im}(:,end+1) = pr';
                rcNoGroups{im}(:,end+1) = rc';
            end
            
            groups = unique(gt);
            groups(groups<=0) = [];
            for im=1:nMethods
                nFns = numel(fns{im});
                pr = zeros(numel(groups),nFns);
                rc = zeros(numel(groups),nFns);
                for k=1:nFns
                    currGroups = splitMatchesToGroups(estPairs{im}{k}(i,j));
                    for ig=groups
                        gtCurr = gt;
                        gtCurr(gtCurr~=ig) = 0;
                        gtCurr(gtCurr>0) = 1;
                        allCurr = all(:,gtCurr==1);
                        
                        % which group has the most matches in common wit GT
                        [~,currIdx] = max(cellfun(...
                            @(m)(sum(ismember(allCurr',m','rows'))),currGroups));
                        curr = currGroups{currIdx};
                        
                        est = ismember(all',curr','rows')';
                        [pr(ig,k),rc(ig,k)] = computePrecisionRecall(gtCurr,est);
                        
                        currGroups{currIdx}=zeros(2,0);
                    end
                end
                prGroups{im}{end+1} = pr';
                rcGroups{im}{end+1} = rc';
            end
        end
    end
end
%% show main curves
figure(1); clf(1); hold on;
plotHandles = zeros(1,nMethods);
for im=1:nMethods
    pr = mean(prNoGroups{im},2);
    rc = mean(rcNoGroups{im},2);
    plotHandles(im) = plotPrecisionRecall(pr,rc,colsMap(methods{im}));
end
legend(plotHandles,methods,'location','southeast');
title('precision-recall averaged over all pairs - all groups considered as one');
xlabel('recall');
ylabel('precision');
xlim([0 1]);
ylim([0 1]);

figure(2); clf(2); hold on;
plotHandles = zeros(1,nMethods);
for im=1:nMethods
    pr = mean(cell2mat(prGroups{im}),2);
    rc = mean(cell2mat(rcGroups{im}),2);
    plotHandles(im) = plotPrecisionRecall(pr,rc,colsMap(methods{im}));
end
legend(plotHandles,methods,'location','southeast');
title('precision-recall averaged over all groups');
xlabel('recall');
ylabel('precision');
xlim([0 1]);
ylim([0 1]);

%% show per group curves
separatorWidthScale = 0.03125;
cams=data.cams;
for pairIdx=1:size(pairsIdxs,2)
    i=pairsIdxs(1,pairIdx);
    j=pairsIdxs(2,pairIdx);
    
    im1 = imread(cams(i).fn);
    im2 = imread(cams(j).fn);
    w1 = size(im1,2);
    h1 = size(im1,1);
    w2 = size(im2,2);
    h2 = size(im2,1);
    offset = w1+ceil(separatorWidthScale*w1);
    img = uint8(zeros(max(h1,h2),offset+w2,size(im1,3)));
    img(1:h1,1:w1,:) = im1;
    img(1:h2,(offset+1):(offset+w2),:) = im2;
    
    matches = data.pairs(i,j).matches;
    keys2 = cams(j).keys;
    for k=1:numel(keys2)
        keys2(k).coord(1) = keys2(k).coord(1)+offset;
    end
    
    figure(5);clf(5);hold on;
    for im=1:nMethods
        nGroups = cellfun(@(p)(numel(p(i,j).groups)),estPairs{im}); 
        plot(1:numel(estPairs{im}),nGroups,'color',colsMap(methods{im}));
    end
    legend(methods);
    
    gt = labels{i,j};
    groups = unique(gt);
    groups(groups<=0) = [];
    for ig=groups
        figure(3); clf(3); hold on
        plotHandles = zeros(1,nMethods);
        for im=1:nMethods
            pr = prGroups{im}{pairIdx}(:,ig);
            rc = rcGroups{im}{pairIdx}(:,ig);
            plotHandles(im) = plotPrecisionRecall(pr,rc,colsMap(methods{im}));
        end
        legend(plotHandles,methods,'location','southeast');
        title(['precision-recall for pair: ' num2str(i) '-' num2str(j)]);
        xlabel('recall');
        ylabel('precision');
        xlim([0 1]);
        ylim([0 1]);
        
        figure(4); clf(4);
        image(img);
        hold on;
        set(gca,'YDir','reverse');
        axis equal;
        axis([0 size(img,2) 0 size(img,1)]);
        
        plotMatches(cams(i).keys,keys2,matches(:,gt==ig),1,[0 1 0],[0 1 0])
        title(['label: ' num2str(ig)]);
        
        key = '';
        while ~strcmp('space',key)
            waitforbuttonpress;
            key=get(gcf,'CurrentKey');
        end
    end
end
%% show gt matches
pairsToShow = pairsIdxs(:,end);
i = pairsToShow(1);
j = pairsToShow(2);

gt = labels{i,j};
pairsCurr = allPairs;
keep = (gt==2) & (pairsCurr(i,j).dists < 0.7);
pairsCurr(i,j).matches(:,~keep) = [];
showLines=1;
showMatches(data.cams,{pairsCurr},0,showLines,pairsToShow,2,1);
    
%% show matches
im=4;
nFns = numel(fns{im});
for k=1:nFns
    pairsCurr = divideMatchesByGroups(estPairs{im}{k});
    showLines=1;
    showMatches(data.cams,pairsCurr,0,showLines,pairsToShow,2,1);
    disp(['k=' num2str(k)]);
    key = '';
    while ~strcmp('space',key)
        waitforbuttonpress;
        key=get(gcf,'CurrentKey');
    end
%     close(gcf);
end