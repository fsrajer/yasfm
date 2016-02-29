function label = plotMatchAndAskForLabel(img,key1,key2,tit)
% convention:
%   label = -1 - don't know -> use spacebar
%   label = 0 - outlier
%   label = 1,2,.. - motion groups

hold off;

image(img);
hold on;
set(gca,'YDir','reverse');
axis equal;
axis([0 size(img,2) 0 size(img,1)]);

markerSize = 15;
lineWidth = 1.5;
plot(key1(1),key1(2), 'g.','MarkerSize', markerSize);
plot(key2(1),key2(2), 'g.','MarkerSize', markerSize);
plot([key1(1) key2(1)],[key1(2) key2(2)],'g','LineWidth',lineWidth);

title(tit);

while true
   isKeyboard = waitforbuttonpress;
   if isKeyboard
       keyPressed = get(gcf,'CurrentCharacter');
       if keyPressed == ' '
           label = -1;
           break;
       elseif ismember(keyPressed,'0123456789')
           label = str2double(keyPressed);
           break;
       end
   end
end

hold off;

end