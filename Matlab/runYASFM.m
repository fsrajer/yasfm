% Filip Srajer, srajefil@fel.cvut.cz
% 30 Dec 2014

addpath('utils');

closeall

fs.DataDirectory      = 'C:\Users\Filip\Workspace\cmp\Data\easttests\';
fs.DatasetSubdirectory      = 'kamen';
fs.imgsSubdir        = 'imgs';
fs.RunEAST           = @fguiRunEAST;

fs.ImgIdxs           = '1 2';
% fs.SimplePoints      = true;
fs.ShowKeypoints     = @fguiShowKeypoints;
fs.ImgPairs            = '1-2';
fs.ShowLines         = false;
fs.ShowAllKeys       = false;
fs.ShowMatchesInit   = false;   
fs.ShowMatchesEG     = false;
% fs.ShowMatchesHG     = false;
fs.ShowMatches       = @fguiShowMatches;
fs.MaxEpLinesInImage = 40;
fs.ShowEpipolarLines = @fguiShowEpipolarLines;
fs.SfMTools          = @fguiShowInSfMTools;
fs.MeshLab           = @fguiShowInMeshLab;

fs.subfigX = 4;
fs.subfigY = 3;

fs.EASTExe            = '..\bin\Incremental.exe';
% fs.SupportDir       = 'C:\Users\Filip\Workspace\cmp\cmp_sfm\cmpsfm_bin';
fs.ListImgs           = 'list_imgs.txt';
% fs.Focals           = 'focal_estimates.txt';
fs.ListKeys           = 'list_keys.txt';
fs.MatchesInit        = 'matches.init.txt';
fs.MatchesEG          = 'matches.eg.txt';
% fs.MatchesHG        = 'matches.hg.txt';
fs.Transforms         = 'transforms.txt';
% fs.Tracks           = 'tracks.txt';
% fs.Constraints      = 'constraints.txt';
% fs.OutputDir        = 'output';
fs.FinalBundleFile    = 'bundle_final.out';
fs.SfmToolsExe        = 'C:\Users\Filip\Workspace\cmp\SFMTools\gui.exe';
fs.FinalPLYFile       = 'sfm.ply';
fs.MeshlabExe         = '"C:\Program Files\VCG\MeshLab\meshlab.exe"';

fs.CloseAll  = @fguiCloseAllClosable;
fs.Exit      = @fguiExit;

fgui(fs);
