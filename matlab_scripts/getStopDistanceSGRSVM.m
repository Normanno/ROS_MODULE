function [ stop_distance ] = getStopDistanceSGRSVM( inputs )
    warning('off', 'all');
    load('matlab.mat');
    stop_distance = getStopDistanceSVM(inputs, numClasses, SVMbottomModelonAllData,SVMupperModelonAllData);
end


