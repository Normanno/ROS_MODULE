function [ stop_distance ] = getStopDistanceSGR( inputs )
    warning('off', 'all');
    load('dataXRun.mat');
    stop_distance = getStopDistanceXRun(inputs, bottomModel,anfisModel);
end


