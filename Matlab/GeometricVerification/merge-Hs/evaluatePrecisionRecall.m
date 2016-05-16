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

fid=fopen('scores-cpp.txt','r');
n=fscanf(fid,'%i',1);
ld3=fscanf(fid,'%lf',[6 n]);
ld3(1:4,:)=ld3(1:4,:)+1;
fclose(fid);

[~,idxs]=sortrows(ld3(1:4,:)');
ld3=ld3(:,idxs);

g=[cell2mat({gt(:).i});cell2mat({gt(:).j});...
    cell2mat({gt(:).ig});cell2mat({gt(:).jg})];
[~,idxs]=sortrows(g');
gt=gt(idxs);

randOrder = randperm(numel(gt));
gt=gt(randOrder);
ld3=ld3(:,randOrder);

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
% methods{end+1}='new - combined2';

scores=[];
% scores = zeros(numel(methods),numel(labels));
% for iH=1:numel(gt)
%     H = inv(gt(iH).H2) * gt(iH).H1;
%     scores(1,iH) = -computeSameEigenvalueDist(H);
% %     scores(2,iH) = computeEigenvectorLineFixedness(H);
% %     scores(3,iH) = computeEigenvectorPointFixedness(H);
% %     scores(1,iH) = -computeSameEigenvalueDistImproved(H);
%     
%     i = gt(iH).i;
%     j = gt(iH).j;
%     keys1=cell2mat({ld.cams(i).keys(:).coord});
%     keys2=cell2mat({ld.cams(j).keys(:).coord});
%     m1 = ld.pairs{gt(iH).ig}(i,j).matches;
%     u11 = keys1(:,m1(1,:));
%     u12 = keys2(:,m1(2,:));
%     m2 = ld.pairs{gt(iH).jg}(i,j).matches;
%     u21 = keys1(:,m2(1,:));
%     u22 = keys2(:,m2(2,:));
%     m = tent(i,j).matches(:,tent(i,j).dists<0.7);
%     m(:,ismember(m',[m1 m2]','rows'))=[];
%     normalizer = 0;%-size(m2,2)-size(m1,2);
%     szs = [];
%     for ig=1:numel(ld.pairs)
%         szs = [szs size(ld.pairs{ig}(i,j).matches,2)];
%         normalizer = normalizer + size(ld.pairs{ig}(i,j).matches,2);
%     end
%     u31 = keys1(:,m(1,:));
%     u32 = keys2(:,m(2,:));
%     [scoreF,scoreG]=computeFundMatErr(gt(iH).H1,gt(iH).H2,u11,u12,u21,u22,u31,u32);
%     nn=size(u11,2)+size(u21,2);
% %     scores(2,iH) = scoreF/nn;
%     scores(2,iH) = (scoreF+scoreG)/nn;%log(scoreF+1)+scoreG/(size(u11,2)+size(u21,2));% * (scoreG / (size(u11,2)+size(u21,2)));
% %     scores(2,iH) = (scoreF+scoreG) / ((size(u11,2)+size(u21,2)) / size(tent(i,j).matches,2));% / (size(u11,2)+size(u21,2));
% %     scores(2,iH) = (scoreF-size(u11,2)-size(u21,2)) / (size(u11,2)+size(u21,2));
%     
%     scores(3,iH) = -scores(2,iH)/scores(1,iH);
% %     scores(4,iH) = ((scores(2,iH)>10)+0.0000001) * scores(1,iH);
% end

scores(end+1,:) = -ld3(end-1,:);
scores(end+1,:) = ld3(end,:);
scores(end+1,:) = ld3(end,:)./ld3(end-1,:);

% figure;
% hold on ;
figure;
vl_roc(labels,scores(1,:))
figure;
vl_roc(labels,scores(2,:))
figure;
vl_roc(labels,scores(end,:))
[tpr,tnr,info] = vl_roc(labels,scores(end,:));
tit = [];
for is=1:size(scores,1)
    figure;
    vl_roc(labels,scores(is,:))
    [tpr,tnr,info] = vl_roc(labels,scores(is,:));
%     [rc, pr, info] = vl_pr(labels,scores(is,:));
%     plot(rc,pr,'linewidth',2,'color', cols(:,is));
%     tit = [tit sprintf('auc%i=%.2f; ',is,info.auc)];
end
% title(tit);

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


