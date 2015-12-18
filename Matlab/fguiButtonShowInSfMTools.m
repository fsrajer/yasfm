function fs = fguiButtonShowInSfMTools(fs, fn)

bundleFn = fullfile(fs.DataDirectory,fs.DatasetSubdirectory,fs.FinalBundleFile);

command = [fs.SfmToolsExe ' ' bundleFn];

system(command);

end