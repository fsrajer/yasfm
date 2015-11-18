function fguiShowMatches(fs, fn)

matchesFilenames={};
cols={};
if fs.ShowMatchesInit
   matchesFilenames{end+1} = fs.MatchesInit;
   cols{1,end+1} = [0 0 1];
   cols{2,end} = [0 0 0.5];
end
if fs.ShowMatchesEG
   matchesFilenames{end+1} = fs.MatchesEG;
   cols{1,end+1} = [0 1 0];
   cols{2,end} = [0 0.5 0];
end

pairsToShow = parseInitPairsToShow(fs.ImgIdxs,fs.ImgPairs);

SimplePoints = true;
showMatches(fullfile(fs.DataDirectory,fs.DatasetSubdirectory), fs.ListImgs, fs.ListKeys, ...
            matchesFilenames,fs.ShowAllKeys,SimplePoints,fs.ShowLines,...
            pairsToShow,fs.subfigX, fs.subfigY,cols);
end

