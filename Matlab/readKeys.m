function keys = readKeys(filename,readDesc)

if nargin < 2
    readDesc = false;
end

if ~strcmp(filename(end-1:end),'gz')
    error('readKeys: unsupported format');
else
    gunzip(filename,'.');
    [~,decompFn,~] = fileparts(filename);
    
    fid = fopen(decompFn,'rb');
    nKeys = fread(fid,1,'int');
    dim = fread(fid,1,'int');
    
    keys = repmat(struct('coord',[0;0],'scale',0,'orientation',0),nKeys,1);
    tmp = reshape(fread(fid,nKeys*4,'float'),[4 nKeys]);
    for i=1:nKeys
        keys(i).coord(2) = tmp(1,i);
        keys(i).coord(1) = tmp(2,i);
        keys(i).scale = tmp(3,i);
        keys(i).orientation = tmp(4,i);
    end
    
    if readDesc
        for i=1:nKeys
            desc = double(fread(fid,dim,'unsigned char'));
            keys(i).desc = desc / norm(desc);
        end
    end
    
    fclose(fid);
    
    delete(decompFn);
end

end