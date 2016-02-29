%% setup
addpath ..
run('../vlfeat-0.9.20/toolbox/vl_setup');

dataDir = 'C:\Users\Filip\Workspace\Data\pairs';
allFn = 'tentatively_matched_all.txt';
outFn = [allFn(1:end-4) '_ground_truth.mat'];

allFn = fullfile(dataDir,allFn);
outFn = fullfile(dataDir,outFn);

%% read results
% ignoreFeats = true;
% data = readResults(allFn,ignoreFeats);
% nImgs = numel(data.cams);
% 
% ld = load(outFn);
% data.labels = ld.labels;

%% show curves

for i=1:nImgs
    for j=(i+1):nImgs
        keys1 = data.cams(i).keys;
        keys2 = data.cams(j).keys;
        
        matches = data.pairs(i,j).matches;
        dists = data.pairs(i,j).dists;
        labels = data.labels{i,j};
        
        matches = matches(:,labels~=-1);
        dists = -dists(labels~=-1);
        labels(labels==-1) = [];
        labels(labels==0) = -1;
        labels(labels>0) = 1;
        n = numel(labels);
        
        if n>0
            ratios = dists;
            
            for iMatch=1:n
                dists(iMatch) = sum((keys1(matches(1,iMatch)).desc - ...
                    keys2(matches(2,iMatch)).desc).^2);
            end
            dists = -dists;
            
            figure;
            vl_pr(labels,ratios);
            figure;
            vl_pr(labels,dists);
            [rc, pr, info] = vl_pr(labels,dists);
        end
    end
end
