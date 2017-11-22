function [ resultsF] = getStopDistanceXRun( inputs, numClasses,  SVM, anfis )
%inputs is a vector with the following variables: Acc.x	Acc.y	Acc.z
%Gyro.x	Gyro.y	Gyro.z	Hr	velocity	Extraversion	Agreeableness
%Conscientiousness	Neuroticism	Openness

resultsClassifier=testSVM(SVM, numClasses, inputs(:,1:8));
    resultsLowLevel=resultsClassifier';

t=[resultsLowLevel inputs(:, 8:13)];

resultsF=evalfis(t,anfis);

end

