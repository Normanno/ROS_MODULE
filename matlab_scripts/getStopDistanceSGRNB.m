function [ stop_distance ] = getStopDistanceSGRNB( inputs )
    warning('off', 'all');
    load('dataXRun.mat');
    stop_distance = getStopDistanceXRun(inputs, bottomModel,anfisModel);
end


