function [ stop_distance ] = getStopDistanceSGR( inputs )
    load('dataXRun.mat');
    stop_distance = getStopDistanceXRun(inputs, bottomModel,anfisModel);
end


