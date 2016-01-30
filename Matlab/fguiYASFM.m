clear fs;
closeall

% Dataset path
fs.DataDirectory      = 'C:\Users\Filip\Workspace\cmp\Data\';
fs.DatasetSubdirectory      = 'daliborka\yasfm';
fs.imgsSubdir        = '..\imgs';

% Running sfm and reading results
fs.YASFMExe            = '..\bin\Incremental.exe';
fs.ccdWidthDb          = '..\resources\camera_ccd_widths.txt';
fs.firstOctave         = -1;
fs.RunYASFM           = @fguiButtonRunYASFM;

fs.ResultsFn1           = 'init.txt';
fs.ResultsFn2           = 'tentatively_matched.txt';
fs.ResultsFn3           = 'matched.txt';
fs.ResultsToReadIdxs     = '2 3';
fs.ReadResults          = @fguiButtonReadResults;

% Visualization
fs.ImgIdxs           = '1 2';
fs.SimplePoints      = false;
fs.ShowKeypoints     = @fguiButtonShowKeypoints;
fs.ImgPairs            = '1-2';
fs.ShowLines         = false;
fs.ShowAllKeys       = false;
fs.ShowMatches       = @fguiButtonShowMatches;
% fs.MaxEpLinesInImage = 40;
% fs.ShowEpipolarLines = @fguiShowEpipolarLines;
fs.SfmToolsExe        = 'C:\Users\Filip\Workspace\cmp\SFMTools\gui.exe';
fs.FinalBundleFile    = 'bundle_final_model0.out';
fs.SfMTools          = @fguiButtonShowInSfMTools;

fs.subfigX = 4;
fs.subfigY = 3;

fs.CloseAll  = @fguiButtonCloseAllClosable;
fs.Exit      = @fguiButtonExit;

fgui(fs);
