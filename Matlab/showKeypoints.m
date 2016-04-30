function showKeypoints(cams, justPts, idxs, subfigX, subfigY)
        
if ~exist('subfigX','var')
    subfigX = 4;
end
if ~exist('subfigY','var')
    subfigY = 3;
end

nCams = numel(cams);

if ~exist('idxs','var')
    idxs=1:nCams;
end

subFigIdx = 1;
for i=idxs
    
    img = imread(cams(i).fn);
    subfig(subfigY, subfigX, mod(subFigIdx-1,subfigX*subfigY) + 1);
    subFigIdx = subFigIdx+1;
    
    hold on;
    image(img);
    set(gca,'YDir','reverse');
    axis equal;
    axis([0 size(img,2) 0 size(img,1)]);
    
    plotKeys(cams(i).keys, justPts);
    
    title(['img: ', num2str(i), ' # keys: ', num2str(numel(cams(i).keys))]);
    hold off
    drawnow
end

end