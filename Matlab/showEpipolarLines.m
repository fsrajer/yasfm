function showEpipolarLines(cams,pairs,pairsToShow,maxLinesInImage,...
    subfigX,subfigY)

if ~exist('subfigX','var')
    subfigX = 4;
end
if ~exist('subfigY','var')
    subfigY = 3;
end

subfigX = floor(subfigX/2);

nCams = numel(cams);

if ~exist('pairsToShow','var') || isempty(pairsToShow)
    pairsToShow = getAllPairs(nCams);
end

imgs = cell(1,nCams);
imgsToRead = unique(pairsToShow(:))';
for i=imgsToRead
   imgs{i} = imread(cams(i).fn); 
end

subfigIdx = 1;
for pairIdxs=pairsToShow
    i=pairIdxs(1);
    j=pairIdxs(2);
    pair = pairs(i,j);
    
    keys1 = cell2mat({cams(i).keys.coord});
    keys2 = cell2mat({cams(j).keys.coord});
    
    off = 0;
    for ig=1:numel(pair.groups)
        g = pair.groups(ig);
        if g.type == 'F'
            subfig(subfigY, subfigX, mod(subfigIdx-1,subfigX*subfigY) + 1);
            subfigIdx = subfigIdx+1;
            
            matches = pair.matches(:,(1:g.size)+off);
            nth = ceil(g.size/maxLinesInImage);
            plotEpipolarLines(imgs{i},imgs{j},keys1(:,matches(1,:)),...
                keys2(:,matches(2,:)),g.T,nth);
            
            title(['pair: ' num2str(i) '-' num2str(j) ...
                ', group: ' num2str(ig)]);
            drawnow;
        end
        off = off + g.size;
    end
end

end
