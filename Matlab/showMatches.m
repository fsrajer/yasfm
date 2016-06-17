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

imgs = cell(1,nCams);
imgsToRead = unique(pairsToShow(:))';
for i=imgsToRead
   imgs{i} = imread(cams(i).fn); 
end

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
    
    tit = ['pair: ' num2str(i) '-' num2str(j) ', # matches:'];
    for iData=1:numel(pairs)
        matches = pairs{iData}(i,j).matches;
        plotMatches(cams(i).keys,keys2,matches,showLines,...
            cols{1,iData},cols{2,iData});
        
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
%     fn = fullfile('C:\Users\Filip\Dropbox\Diplomka\cmpthesis\imgs',fn);
%     export_fig(fn,'-native','-transparent');

end
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
