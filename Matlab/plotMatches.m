function plotMatches(keys1,keys2,matches,...
    showLines,ptColor,lineColor,lineWidth)

if isempty(matches)
    return;
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
        line([keys1(i).coord(1) keys2(j).coord(1)],...
            [keys1(i).coord(2) keys2(j).coord(2)],...
            'Color', lineColor, 'LineWidth', lineWidth);
    end
end

plotKeys(keys1(matches(1,:)),true,ptColor,lineWidth);
plotKeys(keys2(matches(2,:)),true,ptColor,lineWidth);

end