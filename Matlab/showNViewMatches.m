function showNViewMatches(cams,camsIdxs,matches,subfigX,subfigY)

if ~exist('subfigX','var')
    subfigX = 4;
end
if ~exist('subfigY','var')
    subfigY = 3;
end

nCams = numel(cams);
keysIdxs = cell(numel(matches),nCams);
for ir=1:numel(matches)
    for im=1:numel(matches{ir})
        for i=1:numel(matches{ir}{im})
            curr = matches{ir}{im}(i);
            keysIdxs{ir,curr.img}(end+1) = curr.key;
        end
    end
end

cols = initColors();
subFigIdx = 1;
for ic=camsIdxs
    img = imread(cams(ic).fn);
    subfig(subfigY, subfigX, mod(subFigIdx-1,subfigX*subfigY) + 1);
    subFigIdx = subFigIdx+1;
    
    hold on;
    image(img);
    axis ij
    axis equal;
    axis tight
    axis off
    
    tit = ['img: ' num2str(ic) ', # matches: '];
    for ir=1:numel(matches)
        plotKeys(cams(ic).keys(keysIdxs{ir,ic}),true,cols{1,ir});
        tit = [tit num2str(numel(keysIdxs{ir,ic}))  '/'];
    end
    
    tit(end) = [];
    title(tit);
    
    hold off
    drawnow
end

end


function cols = initColors()
cols={};
cols{1,end+1} = [0 1 0];
cols{2,end} = [0 0.5 0];
cols{1,end+1} = [0.6 0.5 0.1]; % brown
cols{2,end} = [0.6 0.5 0.1]; % brown
cols{1,end+1} = [0 0 1];
cols{2,end} = [0 0 0.75];
cols{1,end+1} = [1 0 0];
cols{2,end} = [0.5 0 0];
cols{1,end+1} = [1 0 1];
cols{2,end} = [0.5 0 0.5];
cols{1,end+1} = [0 1 1];
cols{2,end} = [0 0.5 0.5];
cols{1,end+1} = 'k';
cols{2,end} = 'w';
cols{1,end+1} = [1 1 0];
cols{2,end} = [0.5 0.5 0];
cols{1,end+1} = 'w';
cols{2,end} = 'w';

% % cols{1,end+1} = [0.5 0.5 0.5];
% % cols{2,end} = [0.5 0.5 0.5];
% % cols{1,end+1} = [0 0 0];
% % cols{2,end} = [0 0 0];
% cols{1,end+1} = [0 1 0];
% cols{2,end} = [0 0.5 0];
% cols{1,end+1} = [1 0 1];
% cols{2,end} = [0.5 0 0.5];
% cols{1,end+1} = [0 0 1];
% cols{2,end} = [0 0 0.75];
% cols{1,end+1} = [1 0 0];
% cols{2,end} = [0.5 0 0];
% cols{1,end+1} = [1 1 0];
% cols{2,end} = [0.5 0.5 0];
% cols{1,end+1} = [0 1 1];
% cols{2,end} = [0 0.5 0.5];
% cols{1,end+1} = 'k';
% cols{2,end} = 'w';
% cols{1,end+1} = [0.6 0.5 0.1]; % brown
% cols{2,end} = [0.6 0.5 0.1]; % brown
% cols{1,end+1} = 'w';
% cols{2,end} = 'w';
end
