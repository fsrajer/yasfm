Use this scheme:
* master 
 * always stable
* dev 
 * branch where different features get merged
 * can be merged into master whenever all tests pass
* feat/feature_name 
 * feature branches including implementing a feature and its tests, 
 * gets merged into dev when finished
 * when merged into dev and some tests fail, the fixes will be implemented in this branch
* fix/fix_name 
 * bug fix branch
 * gets merged into dev when finished