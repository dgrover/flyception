function plotHeadPos(filename)
% reads log file of timestamps and displays frame rate

formatSpec = '%d %f %f %f %f %f %f %f\n';
sizeA = [8 Inf];

if ~exist('filename', 'var')
    [filename, pathname] = uigetfile('*.txt', 'Select a trajectory txt file');
    fileID = fopen(fullfile(pathname, filename), 'r');
else
    fileID = fopen(filename, 'r');
end

A = fscanf(fileID, formatSpec, sizeA);
A = A';

figure; plot(A(:,4), A(:,5), '.');
xlim([0 256]);
ylim([0 256]);
fclose(fileID);