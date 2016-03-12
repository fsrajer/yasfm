function [pr,rc] = computePrecisionRecall(groundTruth,estimated)

truePositive = sum(groundTruth==1 & estimated==1);
falsePositive = sum(groundTruth==0 & estimated==1);
falseNegative = sum(groundTruth==1 & estimated==0);
pr = truePositive/(truePositive+falsePositive);
rc = truePositive/(truePositive+falseNegative);

xlabel('recall');
ylabel('precision');
xlim([0 1]);
ylim([0 1]);

end