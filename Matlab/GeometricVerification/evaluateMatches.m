%% setup
addpath ..

% to be set by the user
dataDir = 'C:\Users\Filip\Dropbox\pairs';
allFn = 'tentatively_matched_all.txt';
gtFn = [allFn(1:end-4) '_ground_truth.mat'];
% methods = {'ratio','ratio-unique','ratio-unique-gv','ratio-unique-eg'};
% methods = {'ratio','ratio-unique','ratio-unique-eg',...
%     'ratio-unique-eg4x','gv-greedy-group','gv-merge-E'};
methods = {'ratio','ratio-unique','ratio-unique-eg',...
    'eg-E','gv-merge-E'};

allFn = fullfile(dataDir,allFn);
gtFn = fullfile(dataDir,gtFn);
nMethods = numel(methods);
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
ignoreFeats = true;
data = readResults(allFn,ignoreFeats);
allPairs = data.pairs;
nImgs = numel(data.cams);

ld = load(gtFn);
labels = ld.labels;

estPairs = cell(1,nMethods);
for i=1:nMethods
    estPairs{i} = cell(1,numel(fns{i}));
    for j=1:numel(fns{i})
        data = readResults(fullfile(dataDir,methods{i},fns{i}{j}),ignoreFeats);
        estPairs{i}{j} = data.pairs;
    end
end

%% show curves
usedImgs = [];
figIdx = 1;
for i=1:nImgs
    for j=(i+1):nImgs
        gt = labels{i,j};
        
        toUse = (gt ~= -1);
        n = sum(toUse);
        gt(~toUse) = [];
        
        groups = unique(gt);
        groups(groups<=0) = [];
        
        if n>0
            usedImgs = [usedImgs i j];
            titleBase = ['precision-recall for pair ' num2str(i) '-' num2str(j)];
            all = allPairs(i,j).matches(:,toUse);
            
            gtCurr = gt;
            gtCurr(gtCurr>0) = 1; % unite groups
  
            figure(figIdx); hold on;
            plotHandles = zeros(1,nMethods);
            for im=1:nMethods
                nFns = numel(fns{im});
                pr = zeros(1,nFns);
                rc = zeros(1,nFns);
                for k=1:nFns
                    curr = estPairs{im}{k}(i,j).matches;
                    est = ismember(all',curr','rows')';
                    [pr(k),rc(k)] = computePrecisionRecall(gtCurr,est);
                end
                plotHandles(im) = plotPrecisionRecall(pr,rc);
            end
            legend(plotHandles,methods);
            title([titleBase '; all groups']);
            
            for ig=groups
                gtCurr = gt;
                gtCurr(gtCurr~=ig) = 0;
                gtCurr(gtCurr>0) = 1;
                allCurr = all(:,gtCurr==1);
                
                figure(figIdx+ig); hold on;
                for im=1:nMethods
                    nFns = numel(fns{im});
                    pr = zeros(1,nFns);
                    rc = zeros(1,nFns);
                    for k=1:nFns
                        curr = splitMatchesToGroups(estPairs{im}{k}(i,j));
                        
                        % which group has the most matches in common wit GT
                        [~,currIdx] = max(cellfun(@(m)(sum(ismember(allCurr',m','rows'))),curr));
                        curr = curr{currIdx};
                        
                        est = ismember(all',curr','rows')';
                        [pr(k),rc(k)] = computePrecisionRecall(gtCurr,est);
                    end
                    plotHandles(im) = plotPrecisionRecall(pr,rc);
                end
                legend(plotHandles,methods);
                title([titleBase '; group ' num2str(ig)]);
            end
            drawnow;
            figIdx = figIdx + numel(groups) + 1;
        end
    end
end
%% show used images
usedImgs = unique(usedImgs);
n = ceil(sqrt(numel(usedImgs)));
figure;
for i=1:numel(usedImgs)
    subplot(n,n,i);
    imshow(data.cams(usedImgs(i)).fn);
    title(num2str(usedImgs(i)));
end

%% show images
% figure; imshow(data.cams(61).fn);
