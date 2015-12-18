addpath('utils');

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
fs.ResultsToReadIdx     = 1;
fs.ReadResults          = @fguiButtonReadResults;

% Visualization
fs.ImgIdxs           = '1 2';
fs.SimplePoints      = false;
fs.ShowKeypoints     = @fguiButtonShowKeypoints;
% fs.ImgPairs            = '1-2';
% fs.ShowLines         = false;
% fs.ShowAllKeys       = false;
% fs.ShowMatchesInit   = false;   
% fs.ShowMatchesEG     = false;
% % fs.ShowMatchesHG     = false;
% fs.ShowMatches       = @fguiShowMatches;
% fs.MaxEpLinesInImage = 40;
% fs.ShowEpipolarLines = @fguiShowEpipolarLines;
% fs.SfMTools          = @fguiShowInSfMTools;
% fs.MeshLab           = @fguiShowInMeshLab;

fs.subfigX = 4;
fs.subfigY = 3;

% fs.ListImgs           = 'list_imgs.txt';
% % fs.Focals           = 'focal_estimates.txt';
% fs.ListKeys           = 'list_keys.txt';
% fs.MatchesInit        = 'matches.init.txt';
% fs.MatchesEG          = 'matches.eg.txt';
% % fs.MatchesHG        = 'matches.hg.txt';
% fs.Transforms         = 'transforms.txt';
% % fs.Tracks           = 'tracks.txt';
% % fs.Constraints      = 'constraints.txt';
% % fs.OutputDir        = 'output';
% fs.FinalBundleFile    = 'bundle_final.out';
% fs.SfmToolsExe        = 'C:\Users\Filip\Workspace\cmp\SFMTools\gui.exe';
% fs.FinalPLYFile       = 'sfm.ply';
% fs.MeshlabExe         = '"C:\Program Files\VCG\MeshLab\meshlab.exe"';

fs.CloseAll  = @fguiButtonCloseAllClosable;
fs.Exit      = @fguiButtonExit;

fgui(fs);
