clear all
close all
clc

DataBiasRemoved = load('DataBiasRemoved.mat');
DataCalib = load('DataCalib1.mat');
DataWithoutCalib = load('DataWithoutCalib1.mat');
DataWithSetOffsetAndCalib = load('DataWithSetOffsetAndCalib1.mat');
DataZeroSetOffsetWithCalib = load('DataZeroSetOffsetWithCalib1.mat');
DataZeroSetOffsetWithoutCalib = load('DataZeroSetOffsetWithoutCalib1.mat');

dataStructures = {DataBiasRemoved, DataCalib, DataWithoutCalib, DataWithSetOffsetAndCalib, DataZeroSetOffsetWithCalib, DataZeroSetOffsetWithoutCalib};
dataStructuresNames = ["DataBiasRemoved", "DataCalib", "DataWithoutCalib", "DataWithSetOffsetAndCalib", "DataZeroSetOffsetWithCalib", "DataZeroSetOffsetWithoutCalib"];

for i = 1:length(dataStructures)
    
    figure;
    plot(dataStructures{i}.data.Acc(1:3, :)');
    legend("Acc x", "Acc y", "Acc z");
    title(['Acc for Data Structure ', dataStructuresNames(i)]);
    
    figure;
    plot(dataStructures{i}.data.Gyro(1:3, :)');
    legend("Gyro x", "Gyro y", "Gyro z");
    title(['gyro for Data Structure ', dataStructuresNames(i)]);
end