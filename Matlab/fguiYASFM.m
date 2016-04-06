clear fs;
closeall

% Dataset path
fs.DataDirectory      = 'C:\Users\Filip\Dropbox\';
fs.DatasetSubdirectory      = 'pairs';
fs.imgsSubdir        = 'imgs';

% Running sfm and reading results
fs.YASFMExe            = '..\bin\Incremental.exe';
fs.ccdWidthDb          = 'C:\Users\Filip\Workspace\YASFM\resources\camera_ccd_widths.txt';
fs.firstOctave         = -1;
fs.RunYASFM           = @fguiButtonRunYASFM;

fs.ResultsFn1           = 'init.txt';
fs.ResultsFn2           = 'tentatively_matched.txt';
fs.ResultsFn3           = 'matched_1.txt';
fs.ResultsToReadIdxs     = '3';
fs.ReadResults          = @fguiButtonReadResults;

% Visualization
fs.ImgIdxs           = '1';
fs.SimplePoints      = false;
fs.ShowKeypoints     = @fguiButtonShowKeypoints;
fs.ImgPairs            = '1-2';
fs.ShowLines         = false;
fs.ShowAllKeys       = false;
fs.ShowMatches       = @fguiButtonShowMatches;
% fs.MaxEpLinesInImage = 40;
% fs.ShowEpipolarLines = @fguiShowEpipolarLines;
fs.SfmToolsExe        = 'C:\Users\Filip\Workspace\SFMTools\gui.exe';
fs.FinalBundleFile    = 'bundle_final_model0.out';
fs.SfMTools          = @fguiButtonShowInSfMTools;

fs.subfigX = 4;
fs.subfigY = 3;

fs.CloseAll  = @fguiButtonCloseAllClosable;
fs.Exit      = @fguiButtonExit;

fgui(fs);
