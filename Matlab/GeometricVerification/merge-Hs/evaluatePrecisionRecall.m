run('C:/Users/Filip/Workspace/YASFM/matlab/vlfeat-0.9.20/toolbox/vl_setup');

cols=[];
cols(:,end+1) = [213 94 0]/255; % red-ish (vermilion)
cols(:,end+1) = [0 158 115]/255; % bluish green
cols(:,end+1) = [.8 0 .8]; % magenta
cols(:,end+1) = [230 159 0]/255; % orange
cols(:,end+1) = [220 208 66]/255; % dark yellow
cols(:,end+1) = [0 0 0]; % black
cols(:,end+1) = [204 121 167]/255; % pink

% ld=load('HsGT.mat');
gt=ld.HsGT;
% ld2=load('tentative.mat');
tent=ld2.pairs;

gt=gt(randperm(numel(gt)));

% % fid=fopen('Hs.txt','w');
% % fprintf(fid,'%i\n',numel(gt));
% for iH=1:numel(gt)
%     H = inv(gt(iH).H2) * gt(iH).H1;
% %     fprintf(fid,'%.100e ',H(:)');
% %     fprintf(fid,'\n');
% end
% % fclose(fid);


labels = cell2mat({gt(:).isOneMotion});
labels(labels==0) = -1;
methods={};
% methods{end+1}='same eigenvalue dist';
% methods{end+1}='eigenvector line fixedness score';
% methods{end+1}='eigenvector point fixedness score';
% methods{end+1}='eigenvector line fixedness score optimized';
methods{end+1}='same eigenvalue dist improved';
methods{end+1}='new';
methods{end+1}='new - combined';

scores = zeros(numel(methods),numel(labels));
for iH=1:numel(gt)
    H = inv(gt(iH).H2) * gt(iH).H1;
%     scores(1,iH) = -computeSameEigenvalueDist(H);
%     scores(2,iH) = computeEigenvectorLineFixedness(H);
%     scores(3,iH) = computeEigenvectorPointFixedness(H);
    scores(1,iH) = -computeSameEigenvalueDistImproved(H);
    
    i = gt(iH).i;
    j = gt(iH).j;
    keys1=cell2mat({ld.cams(i).keys(:).coord});
    keys2=cell2mat({ld.cams(j).keys(:).coord});
    m = ld.pairs{gt(iH).ig}(i,j).matches;
    u11 = keys1(:,m(1,:));
    u12 = keys2(:,m(2,:));
    m = ld.pairs{gt(iH).jg}(i,j).matches;
    u21 = keys1(:,m(1,:));
    u22 = keys2(:,m(2,:));
    m = tent(i,j).matches;
%     m=zeros(2,0);
%     for ig=1:numel(ld.pairs)
%         if ig == gt(iH).ig || ig == gt(iH).jg
%             continue;
%         end
%         m = [m ld.pairs{ig}(i,j).matches];
%     end
%     normalizer = 0;
%     for ig=1:numel(ld.pairs)
%         normalizer = normalizer + size(ld.pairs{ig}(i,j).matches,2);
%     end
    u31 = keys1(:,m(1,:));
    u32 = keys2(:,m(2,:));
    scoreF=computeFundMatErr(gt(iH).H1,gt(iH).H2,u11,u12,u21,u22,u31,u32);
    scores(2,iH) = (scoreF-size(u11,2)-size(u21,2)) / (size(u11,2)+size(u21,2));
    scores(3,iH) = -scores(2,iH)/scores(1,iH);
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


