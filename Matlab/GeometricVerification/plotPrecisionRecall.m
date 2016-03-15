function plotPrecisionRecall(pr,rc)

plot(rc,pr,'x-','linewidth',1.5);
for k=1:numel(pr)
    text(rc(k),pr(k),num2str(k));
end

end