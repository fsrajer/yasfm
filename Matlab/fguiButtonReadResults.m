function fs = fguiButtonReadResults(fs,fn)

dataDir = fullfile(fs.DataDirectory,fs.DatasetSubdirectory);

switch fs.ResultsToReadIdx
    case 1
        fn = fs.ResultsFn1;
    case 2
        fn = fs.ResultsFn2;
    case 3
        fn = fs.ResultsFn3;
end

if fs.ResultsToReadIdx <= 3
    fs.res = readResults(fullfile(dataDir,fn));
end

end