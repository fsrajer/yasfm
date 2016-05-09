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
    axis ij
    axis equal;
    axis tight
%     
    plotKeys(cams(i).keys, justPts);
    
    title(['img: ', num2str(i), ' # keys: ', num2str(numel(cams(i).keys))]);
    hold off
    drawnow
    
%     axis off
%     title('');
%     % printing
%     set(gca,'Position',get(gca,'OuterPosition'));
%     set(gcf,'PaperUnits','points');
%     sz = size(img);
%     set(gcf,'PaperPosition',[0 0 sz(2) sz(1)]);
%     set(gcf,'PaperSize',[sz(2) sz(1)]);
%     fn = 'sift';
%     print(gcf,fullfile('C:\Users\Filip\Dropbox\Diplomka\cmpthesis\imgs',fn),'-dpdf');
end

end