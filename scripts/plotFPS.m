function plotFPS(filename)
% reads log file of timestamps and displays frame rate

formatSpec = 'Frame %d - TimeStamp %u\n';
sizeA = [2 Inf];

if ~exist('filename', 'var')
    [filename, pathname] = uigetfile('*.txt', 'Select a log txt file');
    fileID = fopen(fullfile(pathname, filename), 'r');
else
    fileID = fopen(filename, 'r');
end

A = fscanf(fileID, formatSpec, sizeA);
A = A';

elap_time = diff(A(:,2));

fps = 1./elap_time * 1000 * 1000;

figure; plot(elap_time, 'o');
figure; plot(fps, 'o');
fclose(fileID);