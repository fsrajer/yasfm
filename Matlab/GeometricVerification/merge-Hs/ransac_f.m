function [Fbest,inl]=ransac_f(u,threshold,confidence)
threshold = threshold^2;
nPts=size(u,2);
num_samples=1000;
sampleSize=7;
Fbest=[];
inl=[];
i=0;
while i < num_samples
   i=i+1;
   idxs=sample(sampleSize,nPts);
   Fcurr_all=u2f7(u(:,idxs));
   if isempty(Fcurr_all)
       continue;
   end
   for j=1:size(Fcurr_all,3)
       Fcurr = Fcurr_all(:,:,j);
       inliers = (fds(Fcurr,u)<threshold);
       if sum(inliers) > sum(inl)
           inl = inliers;
           Fbest = Fcurr;
           num_samples = nsamples(sum(inl), size(u,2), sampleSize, confidence);
       end
   end
end

end