function out = fmfIntensity(infilename,nframes)

if ~exist('infilename', 'var')
    [infilename, pathname] = uigetfile('*.fmf', 'Select fmf file');
    filename = fullfile(pathname, infilename);
else
    filename = infilename;
end

if ~exist( 'nframes', 'var' ) || isempty( nframes )
   nframes = inf; % all
end

% open input file
[header_size, version, f_height, f_width, bytes_per_chunk, max_n_frames, data_format] = fmf_read_header( filename );

if isinf( nframes )
   nframes = max_n_frames;
end

out = zeros(nframes, 1);

fp = fopen(filename,'r');
fseek(fp,header_size,'bof');

wb = waitbar( 0, 'computing pixel intensites' );
for i = 1:nframes,
  waitbar( i/nframes, wb, 'computing pixel intensities' )
  data = fmf_read_frame(fp,f_height,f_width,bytes_per_chunk, data_format);
  
  binaryImage = data >= 140;
  
  out(i) = sum(binaryImage(:)) / (f_height*f_width);
end

plot(out, 'o');

fclose(fp);
close( wb );


