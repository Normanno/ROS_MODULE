function [ stop_distance ] = getStopDistanceSGRSVM( inputs )
    warning('off', 'all');
    if iscell(inputs)
        inputs2=cell2mat(inputs);
    else
        inputs2=inputs;
    end
    load('matlab.mat');
    stop_distance = getStopDistanceSVM(inputs2, numClasses, SVMbottomModelonAllData,SVMupperModelonAllData);
end


