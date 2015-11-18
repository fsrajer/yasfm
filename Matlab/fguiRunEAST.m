function fguiRunEAST(fs, fn)

dataDir = fullfile(fs.DataDirectory,fs.DatasetSubdirectory);

command = [fs.EASTExe ' ' dataDir ' ' fs.imgsSubdir];

system(command);

end