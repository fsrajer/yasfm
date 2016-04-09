function plotEpipolarLines(img1, img2, pts1, pts2, F, nth)

markerSize = 20;
lineWidth = 2;
separatorWidthScale = 0.03125;


w1 = size(img1,2);
h1 = size(img1,1);
w2 = size(img2,2);
h2 = size(img2,1);
offset = w1+ceil(separatorWidthScale*w1);

img = uint8(zeros(max(h1,h2),offset+w2,size(img1,3)));
img(1:h1,1:w1,:) = img1;
img(1:h2,(offset+1):(offset+w2),:) = img2;

hold on;
image(img);
set(gca,'YDir','reverse');
axis equal;
axis([0 size(img,2) 0 size(img,1)]);


idxs = 1:nth:size(pts1,2);
n = numel(idxs);
c = colors( n );

ptsHom = [pts1(:,idxs); 
          ones(1,n); 
          pts2(:,idxs); 
          ones(1,n)];

l1 = F' * ptsHom(4:6,:);
l2 = F * ptsHom(1:3,:); 

for i = 1:n
    plot(pts1(1,idxs(i)), pts1(2,idxs(i)), '.', 'color', c(i,:), 'MarkerSize', markerSize);
    plotLineInImage(w1,h1, l1(:,i),  c(i,:), lineWidth);
end

for i = 1:n
    plot(pts2(1,idxs(i))+offset, pts2(2,idxs(i)), '.', 'color', c(i,:), 'MarkerSize', markerSize);
    plotLineInImage(w2,h2, l2(:,i),  c(i,:),lineWidth,offset);
end

axis off
    
end


    
function c = colors( n )

cl = [ 1   0   0;
       0 0.8   0;
       1   1   0;
       0   0   0;
       0   0.5 0.5;
       0   0   1;
       1   0   1;
      .5   0   .5;
       1   .5  0;

     ];

i = rem( 0:(n-1), size( cl, 1 ) ) + 1;

c = cl( i, : );

end