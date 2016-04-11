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

matchedKeys1 = keys1(matches(1,:));
matchedKeys2 = keys2(matches(2,:));

if showLines
    coord1 = cell2mat({matchedKeys1.coord});
    coord2 = cell2mat({matchedKeys2.coord});
    
    x = [coord1(1,:); coord2(1,:); nan(1,size(matches,2))];
    y = [coord1(2,:); coord2(2,:); nan(1,size(matches,2))];
    
    plot(x(:),y(:),'Color', lineColor, 'LineWidth', lineWidth);
    
%     for i=1:size(matches,2)
%         line([matchedKeys1(i).coord(1) matchedKeys2(i).coord(1)],...
%             [matchedKeys1(i).coord(2) matchedKeys2(i).coord(2)],...
%             'Color', lineColor, 'LineWidth', lineWidth);
%     end
end


plotKeys(matchedKeys1,true,ptColor,lineWidth);
plotKeys(matchedKeys2,true,ptColor,lineWidth);

end