function fguiShowInMeshLab(fs, fn)

plyFn = fullfile(fs.DataDirectory,fs.DatasetSubdirectory,fs.FinalPLYFile);

command = [fs.MeshlabExe ' ' plyFn];

system(command);

end