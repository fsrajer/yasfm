run('c:\workspace\yasfm\Matlab\vlfeat-0.9.20\toolbox\vl_setup.m');
addpath('..\..');

cols=goodCols();

baseFn = 'c:\Users\Uzivatel\Dropbox\cvpr17\0.8\Hs\matched_2';
ld=load([baseFn '-merge_gt.mat']);
gt=ld.gt;
nHs = numel(gt);
g=[cell2mat({gt(:).i});cell2mat({gt(:).j});...
    cell2mat({gt(:).ig});cell2mat({gt(:).jg})];
[~,idxs]=sortrows(g');
gt=gt(idxs);

randOrder = randperm(nHs);
gt=gt(randOrder);
gtLabels = cell2mat({gt(:).isOneMotion});
gtLabels(gtLabels==0) = -1;

methods=[];
methods{end+1} = 'thesis-eig';
methods{end+1} = 'thesis-eg';
methods{end+1} = 'thesis';

for ir=1:numel(methods)
    fid=fopen([baseFn '-labels-' methods{ir} '.txt'],'r');
    res=fscanf(fid,'%lf',[5 nHs]);
    fclose(fid);
    res(1:4,:) = res(1:4,:)+1;
    [~,idxs]=sortrows(res(1:4,:)');
    res=res(:,idxs);
    res=res(:,randOrder);
    scores = res(end,:);
    
    figure(ir);
    vl_roc(gtLabels,scores)
end

