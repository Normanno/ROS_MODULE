function [ resultsF] = getStopDistance( inputs, NB, anfis )
%inputs is a vector with the following variables: Acc.x	Acc.y	Acc.z
%Gyro.x	Gyro.y	Gyro.z	Hr	velocity	Extraversion	Agreeableness
%Conscientiousness	Neuroticism	Openness

%data starts with activity, ends with stop distance



resultsLowLevel=NB.predict(inputs(:,1:8));

t=[resultsLowLevel inputs(:, 8:13)];

resultsF=evalfis(t,anfis); % stop distance



end

