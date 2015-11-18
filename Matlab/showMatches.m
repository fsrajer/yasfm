function showMatches(dir, listImgsFilename, listKeysFilename, ...
                matchesFilenames,showAllKeys,...
                justPts,showLines,...
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

listImgs = readFilenames(fullfile(dir,listImgsFilename));
listKeys = readFilenames(fullfile(dir,listKeysFilename));

nImgs = size(listImgs,2);
if(nImgs ~= size(listKeys,2))
    disp('number of images and number of keypoint files do not match');
    return;
end

if ~exist('pairsToShow','var') || isempty(pairsToShow)
    pairsToShow = getAllPairs(nImgs);
elseif size(pairsToShow,1)==1
    idxs=pairsToShow;
    pairsToShow = getAllPairs(nImgs);
    toKeep = ismember(pairsToShow(1,:),idxs) | ismember(pairsToShow(2,:),idxs);
    pairsToShow=pairsToShow(:,toKeep);
end

imgs = readImgs(dir,listImgs,pairsToShow);
keys = readKeysAll(dir,listKeys,pairsToShow);

nMatchFiles=size(matchesFilenames,2);
matches=cell(1,nMatchFiles);
for i=1:nMatchFiles
    matches{i} = readMatches(fullfile(dir,matchesFilenames{i}),nImgs);
end

subfigIdx = 1;
for pair=pairsToShow
    i=pair(1);
    j=pair(2);
    
    allempty = true;
    for k=1:nMatchFiles
        if ~isempty(matches{k}{i,j})
            allempty=false;
        end
    end
    if allempty
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

    keys2 = keys{j};
    keys2(1,:) = keys2(1,:)+offset;
    
    if showAllKeys
        plotKeys(keys{i},justPts,'y');
        plotKeys(keys2,justPts,'y');
    end
    
    tit = ['pair: ' num2str(i) '-' num2str(j) ', # matches:'];
    for k=1:nMatchFiles
        l = mod(k-1,size(cols,2))+1;
        plotMatches(keys{i},keys2,matches{k}{i,j},justPts,showLines,cols{1,l},cols{2,l});
        tit = [tit ' ' num2str(size(matches{k}{i,j},2)) ','];
    end
    tit = tit(1:end-1);
    title(tit);
    
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
