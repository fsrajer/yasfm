function keys = readKeys(filename)

if ~strcmp(filename(end-1:end),'gz')
    error('readKeys: unsupported format');
else
    gunzip(filename,'.');
    [~,decompFn,~] = fileparts(filename);
    
    fid = fopen(decompFn,'rb');
    nKeys = fread(fid,1,'int');
    dim = fread(fid,1,'int');
    
    keys = repmat(struct('coord',[0;0],'scale',0,'orientation',0),nKeys,1);
    for i=1:nKeys
        keys(i).coord(2) = fread(fid,1,'float');
        keys(i).coord(1) = fread(fid,1,'float');
        keys(i).scale = fread(fid,1,'float');
        keys(i).orientation = fread(fid,1,'float');
    end
    fclose(fid);
    
    delete(decompFn);
end

end