function fs = fguiButtonReadResults(fs,fn)

dataDir = fullfile(fs.DataDirectory,fs.DatasetSubdirectory);

idxs = str2double(strsplit(fs.ResultsToReadIdxs));
fns = cell(numel(idxs),1);
for i=1:numel(idxs)
    switch idxs(i)
        case 1
            fns{i} = fs.ResultsFn1;
        case 2
            fns{i} = fs.ResultsFn2;
        case 3
            fns{i} = fs.ResultsFn3;
    end
end

fs.res = cell(1,numel(fns));
ignoreFeats = false;
for i=1:numel(fns)
    fs.res{i} = readResults(fullfile(dataDir,fns{i}),ignoreFeats);
    ignoreFeats = true;
end

end