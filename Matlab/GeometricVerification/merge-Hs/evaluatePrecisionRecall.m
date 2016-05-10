run('C:/Users/Filip/Workspace/YASFM/matlab/vlfeat-0.9.20/toolbox/vl_setup');

cols=[];
cols(:,end+1) = [213 94 0]/255; % red-ish (vermilion)
cols(:,end+1) = [0 158 115]/255; % bluish green
cols(:,end+1) = [.8 0 .8]; % magenta
cols(:,end+1) = [230 159 0]/255; % orange
cols(:,end+1) = [220 208 66]/255; % dark yellow
cols(:,end+1) = [0 0 0]; % black
cols(:,end+1) = [204 121 167]/255; % pink

ld=load('HsGT.mat');
gt=ld.HsGT;

gt=gt(randperm(numel(gt)));

% fid=fopen('Hs.txt','w');
% fprintf(fid,'%i\n',numel(gt));
for iH=1:numel(gt)
    H = inv(gt(iH).H2) * gt(iH).H1;
%     fprintf(fid,'%.100e ',H(:)');
%     fprintf(fid,'\n');
end
% fclose(fid);


labels = cell2mat({gt(:).isOneMotion});
labels(labels==0) = -1;
methods={};
methods{end+1}='my first';
methods{end+1}='eigenvector line fixedness score';
% methods{end+1}='eigenvector point fixedness score';
% methods{end+1}='eigenvector line fixedness score optimized';
methods{end+1}='my second';

scores = zeros(numel(methods),numel(labels));
for iH=1:numel(gt)
    H = inv(gt(iH).H2) * gt(iH).H1;
    scores(1,iH) = -computeScoreMyFirst(H);
    scores(2,iH) = computeEigenvectorLineFixedness(H);
%     scores(3,iH) = computeEigenvectorPointFixedness(H);
    scores(3,iH) = -computeScoreMySecond(H);
end

% fid=fopen('Hs-cpplabels_.txt','r');
% n=fscanf(fid,'%i',1);
% scores(4,:) = fscanf(fid,'%lf',[n 1]);
% fclose(fid);

figure;
hold on ;

tit = [];
for is=1:size(scores,1)
    [rc, pr, info] = vl_pr(labels,scores(is,:));
    plot(rc,pr,'linewidth',2,'color', cols(:,is));
    tit = [tit sprintf('auc%i=%.2f; ',is,info.auc)];
end
title(tit);

p = sum(labels>0);
n = sum(labels<0);
randomPrecision = p / (p + n) ;
spline([0 1], [1 1] * randomPrecision, 'r--', 'linewidth', 2) ;

axis square ; grid on ;
xlim([0 1]) ; 
xlabel('recall') ;
% xlabel('recall','fontsize',18) ;
ylim([0 1]) ; 
ylabel('precision') ;
% ylabel('precision','fontsize',18) ;
% set(gca,'fontsize',18)

% title(sprintf('PR (AUC: %.2f%%, AP: %.2f%%, AP11: %.2f%%)', ...
%     info.auc * 100, ...
%     info.ap * 100, ...
%     info.ap_interp_11 * 100)) ;
legend(methods,'location','se');


