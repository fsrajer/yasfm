%% setup
addpath ..

% to be set by the user
dataDir = 'C:\Users\Filip\Dropbox\pairs';
allFn = 'tentatively_matched_all.txt';
gtFn = [allFn(1:end-4) '_ground_truth.mat'];
names = {'ratio','ratio-unique','ratio-unique-gv','ratio-unique-eg'};

allFn = fullfile(dataDir,allFn);
gtFn = fullfile(dataDir,gtFn);
nNames = numel(names);
fns = cell(1,nNames);
matlabDir = pwd;
for i=1:nNames
    d = fullfile(dataDir,names{i});
    cd(d);
    fns{i} = cellstr(ls('matched*'));
    
    % sort based on numbers
    r = regexp(fns{i},'\d+','match'); %find numbers
    [~, ind] = sort(cellfun(@(c) str2double(c{1}), r)); % convert to numbers and sort
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

filteredPairs = cell(1,nNames);
for i=1:nNames
    filteredPairs{i} = cell(1,numel(fns{i}));
    for j=1:numel(fns{i})
        data = readResults(fullfile(dataDir,names{i},fns{i}{j}),ignoreFeats);
        filteredPairs{i}{j} = data.pairs;
    end
end

%% show curves
usedImgs = [];
for i=1:nImgs
    for j=(i+1):nImgs
        gt = labels{i,j};
        toUse = (gt ~= -1);
        n = sum(toUse);
        
        gt(~toUse) = [];
        gt(gt>0) = 1; % unite groups
        
        if n>0
            usedImgs = [usedImgs i j];
            all = allPairs(i,j).matches(:,toUse);
            figure;
            hold on
            for iname=1:nNames
                nFns = numel(fns{iname});
                precision = zeros(1,nFns);
                recall = zeros(1,nFns);
                for k=1:nFns
                    curr = filteredPairs{iname}{k}(i,j).matches;
                    est = ismember(all',curr','rows')';
                    truePositive = sum(gt==1 & est==1);
                    falsePositive = sum(gt==0 & est==1);
                    falseNegative = sum(gt==1 & est==0);
                    precision(k) = truePositive/(truePositive+falsePositive);
                    recall(k) = truePositive/(truePositive+falseNegative);
                end
                plot(recall,precision,'x-','linewidth',1.5);
                for k=1:nFns
                    text(recall(k),precision(k),num2str(k));
                end
            end
            legend(names);
            title(['precision-recall for pair ' num2str(i) '-' num2str(j)])
            xlabel('recall');
            ylabel('precision');
            xlim([0 1]);
            ylim([0 1]);
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
