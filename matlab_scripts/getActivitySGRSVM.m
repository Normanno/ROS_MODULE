function [ activity ] = getActivitySGRSVM( inputs )
    warning('off', 'all');

    if iscell(inputs)
        inputs2=cell2mat(inputs);
    else
        inputs2=inputs;
    end
    load('matlab.mat');
    activity=testSVM(SVMbottomModelonAllData, numClasses, inputs2(:,1:8));
end


