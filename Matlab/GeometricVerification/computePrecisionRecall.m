function [pr,rc] = computePrecisionRecall(groundTruth,estimated)

truePositive = sum(groundTruth==1 & estimated==1);
falsePositive = sum(groundTruth==0 & estimated==1);
falseNegative = sum(groundTruth==1 & estimated==0);
pr = truePositive/(truePositive+falsePositive);
if truePositive+falsePositive == 0
    pr = 1;
end
rc = truePositive/(truePositive+falseNegative);

end