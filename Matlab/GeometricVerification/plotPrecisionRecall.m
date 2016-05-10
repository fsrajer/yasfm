function h = plotPrecisionRecall(pr,rc,col,scale)
if nargin < 4
    scale = 1;
end
h = plot(rc,pr,'x-','linewidth',1.5*scale,'color',col);
if(numel(pr)>=33)
    plot(rc([25 29 33]),pr([25 29 33]),...
        'x','markersize',20*scale,'linewidth',5*scale,'color',col);
end
axis equal

for k=1:numel(pr)
    text(rc(k),pr(k),num2str(k));
end

end