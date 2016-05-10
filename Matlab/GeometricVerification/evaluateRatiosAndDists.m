%% setup
addpath ..
run('../vlfeat-0.9.20/toolbox/vl_setup');

dataDir = 'C:\Users\Filip\Dropbox\pairs';
allFn = 'tentatively_matched_all_with_gt.txt';
outFn = 'tentatively_matched_all_ground_truth.mat';

allFn = fullfile(dataDir,allFn);
outFn = fullfile(dataDir,outFn);

%% read results
% ignoreFeats = false;
% readDesc = true;
% data = readResults(allFn,ignoreFeats,readDesc);
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
            hold on ;
            [rc, pr, info] = vl_pr(labels,dists);
            plot(rc,pr,'linewidth',2,'color', [213 94 0]/255) ;
            [rc, pr, info] = vl_pr(labels,ratios);
            plot(rc,pr,'linewidth',2,'color', [0 114 178]/255) ;
%             randomPrecision = p / (p + n) ;
%             spline([0 1], [1 1] * randomPrecision, 'r--', 'linewidth', 2) ;
            axis square ; grid on ;
            xlim([0 1]) ; xlabel('recall','fontsize',18) ;
            ylim([0 1]) ; ylabel('precision','fontsize',18) ;
            set(gca,'fontsize',18)
%             title(sprintf('PR (AUC: %.2f%%, AP: %.2f%%, AP11: %.2f%%)', ...
%                 info.auc * 100, ...
%                 info.ap * 100, ...
%                 info.ap_interp_11 * 100)) ;
            legend('descriptor distance','Lowe ratio');
%             vl_pr(labels,ratios);
%             figure;
%             vl_pr(labels,dists);
%             [rc, pr, info] = vl_pr(labels,dists);
        end
    end
end
