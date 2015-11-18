function showKeypoints(dir, listImgsFilename, listKeysFilename, justPts, ...
                    idxs, subfigX, subfigY)

if ~exist('subfigX','var')
    subfigX = 4;
end
if ~exist('subfigY','var')
    subfigY = 3;
end
                
imgFns = readFilenames(fullfile(dir,listImgsFilename));
keysFns = readFilenames(fullfile(dir,listKeysFilename));

nImgs = size(imgFns,2);
if(nImgs ~= size(keysFns,2))
    disp('number of images and number of keypoint files do not match');
    return;
end

if ~exist('idxs','var')
    idxs=1:nImgs;
end

subFigIdx = 1;
for i=idxs
    
    img = imread(fullfile(dir,imgFns{i}));
    subfig(subfigY, subfigX, mod(subFigIdx-1,subfigX*subfigY) + 1);
    subFigIdx = subFigIdx+1;
    
    hold on;
    image(img);
    set(gca,'YDir','reverse');
    axis equal;
    axis([0 size(img,2) 0 size(img,1)]);
    
    if justPts
        feat = readKeys(fullfile(dir,keysFns{i}));
    else
        feat = readFeats(fullfile(dir,keysFns{i}));
    end
    plotKeys(feat, justPts);
    
    title(['img: ', num2str(i), ' # keys: ', num2str(size(feat,2))]);
    hold off
end

end