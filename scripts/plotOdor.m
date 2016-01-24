function plotOdor(filename)
% reads log file of timestamps and displays frame rate

formatSpec = '%d %f %f %f %f %f %f %f %f %d\n';
sizeA = [10 Inf];

if ~exist('filename', 'var')
    [filename, pathname] = uigetfile('*.txt', 'Select a trajectory txt file');
    fileID = fopen(fullfile(pathname, filename), 'r');
else
    fileID = fopen(filename, 'r');
end

A = fscanf(fileID, formatSpec, sizeA);
A = A';

figure; plot(A(:,10), '.');
fclose(fileID);