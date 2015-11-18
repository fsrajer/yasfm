function plotMatches(keys1,keys2,matches,justPts,showLines,ptColor,lineColor,lineWidth)

if isempty(matches)
    return;
end

if ~exist('matches','var') && (size(keys1,2)==size(keys2,2))
    matches = 1:size(keys1,2);
    matches = [matches; matches];
end
if ~exist('justPts','var')
    justPts = true;
end
if ~exist('showLines','var')
    showLines = true;
end
if ~exist('ptColor','var')
    ptColor = 'b';
end
if ~exist('lineColor','var')
    lineColor = ptColor;
end
if ~exist('lineWidth','var')
    lineWidth = 1;%1.25;2.5; 4
end

for idx=1:size(matches,2)
    i=matches(1,idx);
    j=matches(2,idx);
    if showLines
        line([keys1(1,i) keys2(1,j)], [keys1(2,i) keys2(2,j)],...
            'Color', lineColor, 'LineWidth', lineWidth);
    end
end

plotKeys(keys1(:,matches(1,:)),justPts,ptColor,lineWidth);
plotKeys(keys2(:,matches(2,:)),justPts,ptColor,lineWidth);

end