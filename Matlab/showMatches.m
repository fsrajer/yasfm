function showMatches(data,showAllKeys,showLines,...
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

nCams = numel(data.cams);

if ~exist('pairsToShow','var') || isempty(pairsToShow)
    pairsToShow = getAllPairs(nCams);
end

imgs = cell(1,nCams);
imgsToRead = unique(pairsToShow(:))';
for i=imgsToRead
   imgs{i} = imread(data.cams(i).fn); 
end

subfigIdx = 1;
for pair=pairsToShow
    i=pair(1);
    j=pair(2);
    
    matches = data.pairs(i,j).matches;
    if isempty(matches)
        continue;
    end
    
    w1 = size(imgs{i},2);
    h1 = size(imgs{i},1);
    w2 = size(imgs{j},2);
    h2 = size(imgs{j},1);
    offset = w1+ceil(separatorWidthScale*w1);
    
    img = uint8(zeros(max(h1,h2),offset+w2,size(imgs{i},3)));
    img(1:h1,1:w1,:) = imgs{i};
    img(1:h2,(offset+1):(offset+w2),:) = imgs{j};
    
    subfig(subfigY, subfigX, mod(subfigIdx-1,subfigX*subfigY) + 1);
    subfigIdx = subfigIdx+1;

    hold on;
    image(img);
    set(gca,'YDir','reverse');
    axis equal;
    axis([0 size(img,2) 0 size(img,1)]);

    keys2 = data.cams(j).keys;
    for k=1:numel(keys2)
        keys2(k).coord(1) = keys2(k).coord(1)+offset;
    end
    
    if showAllKeys
        plotKeys(data.cams(i).keys,true,'y');
        plotKeys(keys2,true,'y');
    end
    
    title(['pair: ' num2str(i) '-' num2str(j) ', # matches:' ...
        num2str(size(matches,2))]);
    
    plotMatches(data.cams(i).keys,keys2,matches,showLines,...
        cols{1,2},cols{2,2});
    
    axis off
    hold off
end

end

function cols = initColors()
cols=cell(2,3);
cols{1,1} = [0 0 1];
cols{2,1} = [0 0 0.5];
cols{1,2} = [0 1 0];
cols{2,2} = [0 0.5 0];
cols{1,3} = 'k';
cols{2,3} = 'w';
end
