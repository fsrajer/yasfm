function plotKeys(feat,justPts,color,lineWidth,markerSize)

if ~exist('justPts','var')
    justPts = true;
end
if ~exist('color','var')
    color = [1 1 0];
end
if ~exist('lineWidth','var')
    lineWidth = 2;
end
if ~exist('markerSize','var')
    markerSize = 10;%15;25;50
end

if justPts
    plot(feat(1,:), feat(2,:), 'g.', 'MarkerSize', markerSize, 'Color', color);
    
else
    nPoints = 40;
    nKeys = size(feat,2);
    XRes = nan*zeros(2, nPoints*nKeys + (nKeys-1));
    OriRes = nan*zeros(2, 2*nKeys + (nKeys-1));
    
    temp = linspace(0,2*pi,nPoints); 
    Xcirc = [ cos(temp); sin(temp) ];
    for i=1:nKeys
        r = feat(3,i); %scale -> radius 
        c = feat(1:2,i); %center
        X = Xcirc*r + repmat(c,1,nPoints);
        ori = [ cos(feat(4,i)); sin(feat(4,i)) ] * r;
        XRes(:, (i-1)*(nPoints+1) + (1:nPoints)) = X;
        OriRes(:, (i-1)*3 + (1:2)) = repmat(c,1,2) + [[0; 0] ori];
    end
    
    line([XRes(1,:) nan OriRes(1,:)], [XRes(2,:) nan OriRes(2,:)],...
        'Color', color,'LineWidth', lineWidth);
end

end