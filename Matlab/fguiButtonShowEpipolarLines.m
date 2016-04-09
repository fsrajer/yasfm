function fs = fguiButtonShowEpipolarLines(fs, fn)

pairsToShow = parseInitPairsToShow(fs.ImgIdxs,fs.ImgPairs,...
    numel(fs.res{1}.cams));

for i=1:numel(fs.res)
    curr = fs.res{i};
    if isfield(curr.pairs,'groups') 
        maxTrans = max(cellfun(@numel,{curr.pairs.groups}));
        if maxTrans > 0
           showEpipolarLines(curr.cams,curr.pairs,pairsToShow,...
            fs.MaxEpLinesInImage,fs.subfigX,fs.subfigY); 
        end
    end
end

end