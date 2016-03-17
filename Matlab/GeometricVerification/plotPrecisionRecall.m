function h = plotPrecisionRecall(pr,rc)

colorOrder = get(gca, 'ColorOrder');
colIdx = mod(length(get(gca, 'Children'))/42, size(colorOrder, 1))+1;
col = colorOrder(colIdx, :);

h = plot(rc,pr,'x-','linewidth',1.5,'color',col);
plot(rc([25 29 33]),pr([25 29 33]),'x','markersize',20,'linewidth',5,'color',col);

for k=1:numel(pr)
    text(rc(k),pr(k),num2str(k));
end

end