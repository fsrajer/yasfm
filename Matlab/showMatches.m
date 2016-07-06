function showMatches(cams,pairs,showAllKeys,showLines,...
    pairsToShow,subfigX,subfigY,cols)

if ~exist('subfigX','var')
    subfigX = 4;
end
if ~exist('subfigY','var')
    subfigY = 3;
end
if ~exist('cols','var')
    cols = initColors();
end

subfigX = floor(subfigX/2);
separatorWidthScale = 0.03125;

nCams = numel(cams);

if ~exist('pairsToShow','var') || isempty(pairsToShow)
    pairsToShow = getAllPairs(nCams);
end

nMatchesTotal = zeros(size(pairsToShow,2),1);
for iPair=1:numel(nMatchesTotal)
    i=pairsToShow(1,iPair);
    j=pairsToShow(2,iPair);
    for iData=1:numel(pairs)
        nMatchesTotal(iPair) = nMatchesTotal(iPair) + ...
            size(pairs{iData}(i,j).matches,2);
    end
end
pairsToShow(:,nMatchesTotal==0) = [];
nMatchesTotal(nMatchesTotal==0) = [];

HsGT = repmat(struct('H1',[],'H2',[],'i',[],'j',[],'ig',[],'jg',[]),0,1);

imgs = cell(1,nCams);
imgsToRead = unique(pairsToShow(:))';
for i=imgsToRead
   imgs{i} = imread(cams(i).fn); 
end
% %=== ground truth
% ignoreFeats = true;
% data = readResults('C:\Users\Filip\Dropbox\pairs\tentatively_matched_all.txt',ignoreFeats);
% allPairs = data.pairs;
% load('C:\Users\Filip\Dropbox\pairs\tentatively_matched_all_ground_truth.mat');
% %=== 
subfigIdx = 1;
for pair=pairsToShow
    i=pair(1);
    j=pair(2);
    
    w1 = size(imgs{i},2);
    h1 = size(imgs{i},1);
    w2 = size(imgs{j},2);
    h2 = size(imgs{j},1);
    offset = w1+ceil(separatorWidthScale*w1);
    
    img = uint8(255+zeros(max(h1,h2),offset+w2,size(imgs{i},3)));
    img(1:h1,1:w1,:) = imgs{i};
    img(1:h2,(offset+1):(offset+w2),:) = imgs{j};
    
    subfig(subfigY, subfigX, mod(subfigIdx-1,subfigX*subfigY) + 1);
    subfigIdx = subfigIdx+1;

    hold on;
    image(img);
    axis equal;
    axis ij;
    axis tight;
    axis off;

    keys2 = cams(j).keys;
    for k=1:numel(keys2)
        keys2(k).coord(1) = keys2(k).coord(1)+offset;
    end
    
    if showAllKeys
        plotKeys(cams(i).keys,true,'y');
        plotKeys(keys2,true,'y');
    end
    
%     for ig=1:numel(pairs)
%         for jg=(ig+1):numel(pairs)
%             
%             iMatches = pairs{ig}(i,j).matches;
%             jMatches = pairs{jg}(i,j).matches;
%             
%             if(size(iMatches,2) == 0 || size(jMatches,2) == 0)
%                 continue;
%             end
%             
%             plotMatches(cams(i).keys,keys2,iMatches,showLines,...
%                 cols{1,1},cols{2,1});
%             
%             plotMatches(cams(i).keys,keys2,jMatches,showLines,...
%                 cols{1,2},cols{2,2});
%             
%             while true
%                 isKeyboard = waitforbuttonpress;
%                 if isKeyboard
%                     keyPressed = get(gcf,'CurrentCharacter');
%                     if ismember(keyPressed,'01')
%                         label = str2double(keyPressed);
%                         break;
%                     elseif strcmp('escape',get(gcf,'CurrentKey'))
%                         label = Inf;
%                         break;
%                     end
%                 end
%             end
%             HsGT(end+1).H1 = pairs{ig}(i,j).groups.T;
%             HsGT(end).H2 = pairs{jg}(i,j).groups.T;
%             HsGT(end).i = i;
%             HsGT(end).j = j;
%             HsGT(end).ig = ig;
%             HsGT(end).jg = jg;
%             HsGT(end).isOneMotion = label;
%             
%             save('HsGT.mat','HsGT');
%             
%             clf
%             hold on;
%             image(img);
%             axis equal;
%             axis ij;
%             axis tight;
%             axis off;
%         end
%     end
    
    tit = ['pair: ' num2str(i) '-' num2str(j) ', # matches:'];
    for iData=1:numel(pairs)
        matches = pairs{iData}(i,j).matches;
        plotMatches(cams(i).keys,keys2,matches,showLines,...
            cols{1,iData},cols{2,iData});
        
%         %=== ground truth
%         if ~isempty(matches)
%             gtCurr = labels{i,j}';
%             all = allPairs(i,j).matches;
%             [est,all2curr] = ismember(all',matches','rows');
%             bad = all2curr(gtCurr==0 & est>0);
%             bad(bad==0) = [];
%             if ~isempty(bad)
%                 keys1 = cams(i).keys;
%                 coord1 = cell2mat({keys1(matches(1,bad)).coord});
%                 coord2 = cell2mat({keys2(matches(2,bad)).coord});
%                 plot(coord1(1,:),coord1(2,:),'ro');
%                 plot(coord2(1,:),coord2(2,:),'ro');
%             end
%         end
%         %===
        
        tit = [tit num2str(size(matches,2)) '/'];
    end
    
    tit(end) = [];
    title(tit);
    
    hold off
    drawnow;
    
%     title('');
%     set(gca, 'Color', 'none');
%     addpath export_fig
%     fn = sprintf('seq-ransac-F-%i-%i.pdf',i,j);
%     fn = fullfile('C:\Users\Uzivatel\Dropbox\cvpr17\figs',fn);
%     export_fig(fn,'-native','-transparent');

end
% save('HsGT.mat','HsGT','cams','pairs');
end

function cols = initColors()
cols={};
% cols{1,end+1} = [0.5 0.5 0.5];
% cols{2,end} = [0.5 0.5 0.5];
% cols{1,end+1} = [0 0 0];
% cols{2,end} = [0 0 0];
cols{1,end+1} = [0 1 0];
cols{2,end} = [0 0.5 0];
cols{1,end+1} = [1 0 1];
cols{2,end} = [0.5 0 0.5];
cols{1,end+1} = [0 0 1];
cols{2,end} = [0 0 0.75];
cols{1,end+1} = [1 0 0];
cols{2,end} = [0.5 0 0];
cols{1,end+1} = [1 1 0];
cols{2,end} = [0.5 0.5 0];
cols{1,end+1} = [0 1 1];
cols{2,end} = [0 0.5 0.5];
cols{1,end+1} = 'k';
cols{2,end} = 'w';
cols{1,end+1} = 'm';
cols{2,end} = 'm';
end
