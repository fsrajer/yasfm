function h = plotPrecisionRecall(pr,rc,col)

h = plot(rc,pr,'x-','linewidth',1.5,'color',col);
if(numel(pr)>=33)
    plot(rc([25 29 33]),pr([25 29 33]),...
        'x','markersize',20,'linewidth',5,'color',col);
end

for k=1:numel(pr)
    text(rc(k),pr(k),num2str(k));
end

end