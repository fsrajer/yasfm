function plotLineInImage(w, h, line, col, lineWidth,xoffset)

if ~exist('xoffset','var')
    xoffset = 0;
end

drawLineInBox(line, [1 1], [w 1], [w h],...
    [1 h], col, lineWidth,xoffset);

end


function boxIsect = drawLineInBox(l, lt, rt, rd, ld, col, lineWidth,xoffset)

boxIsect = getBoxIsect(l, lt, rt, rd, ld);

if size(boxIsect,2) >= 2
    plot(boxIsect(1,:)+xoffset, boxIsect(2,:), 'color', col, 'LineWidth', lineWidth);
end

end

function boxIntersection = getBoxIsect(l, lt, rt, rd, ld)

horizontal = [1; 0];
vertical = [0; 1];
left = [horizontal; -lt(1)];
right = [horizontal; -rt(1)];
top = [vertical; -lt(2)];
bottom = [vertical; -ld(2)];

idx = 1;
pts(:,idx) = isectLines(l, left);
if isInImage(pts(:,idx), lt, rd)
    idx = idx+1;
end
pts(:,idx) = isectLines(l, top);
if isInImage(pts(:,idx), lt, rd)
    idx = idx+1;
end
pts(:,idx) = isectLines(l, right);
if isInImage(pts(:,idx), lt, rd)
    idx = idx+1;
end
pts(:,idx) = isectLines(l, bottom);

boxIntersection = pts(:,1:2);

end

function res = isInImage(pts, lt, rd)

myeps = 1e-10;
a = pts(1,:)-lt(1);
b = pts(2,:)-lt(2);
c = rd(1)-pts(1,:);
d = rd(2)-pts(2,:);
res = (a > -myeps) & (b > -myeps) & (c > -myeps) & (d > -myeps);

% res = (pts(1,:) >= lt(1)) & (pts(1,:) <= rd(1)) ...
%         & (pts(2,:) >= lt(2)) & (pts(2,:) <= rd(2));
end