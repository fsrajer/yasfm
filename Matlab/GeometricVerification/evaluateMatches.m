%% setup
addpath ..

dataDir = 'C:\Users\Filip\Workspace\Data\pairs';
allFn = 'tentatively_matched_all.txt';
gtFn = [allFn(1:end-4) '_ground_truth.mat'];
baseFn = 'matched_ratio';
nFns = 5;

allFn = fullfile(dataDir,allFn);
gtFn = fullfile(dataDir,gtFn);
fns = cell(1,nFns);
for i=1:nFns
    fns{i} = fullfile(dataDir,[baseFn num2str(i) '.txt']);
end

%% read data
ignoreFeats = true;
data = readResults(allFn,ignoreFeats);
allPairs = data.pairs;
nImgs = numel(data.cams);

ld = load(gtFn);
labels = ld.labels;

filteredPairs = cell(1,nFns);
for i=1:nFns
    data = readResults(fns{i},ignoreFeats);
    filteredPairs{i} = data.pairs;
end

%% show curves
for i=1:nImgs
    for j=(i+1):nImgs
        gt = labels{i,j};
        toUse = (gt ~= -1);
        n = sum(toUse);
        
        gt(~toUse) = [];
        gt(gt>0) = 1; % unite groups
        
        if n>0
            all = allPairs(i,j).matches(:,toUse);
            precision = zeros(1,nFns);
            recall = zeros(1,nFns);
            for k=1:nFns
                curr = filteredPairs{k}(i,j).matches;
                est = ismember(all',curr','rows')';
                truePositive = sum(gt==1 & est==1);
                falsePositive = sum(gt==0 & est==1);
                falseNegative = sum(gt==1 & est==0);
                precision(k) = truePositive/(truePositive+falsePositive);
                recall(k) = truePositive/(truePositive+falseNegative);
            end
            figure;
            plot(recall,precision,'bx-','linewidth',1.5);
            title('precision-recall')
            xlabel('recall');
            ylabel('precision');
            xlim([0 1]);
            ylim([0 1]);
        end
    end
end
