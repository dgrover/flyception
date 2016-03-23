%Drosophilia courtship song simulator
%
%   Generates a song consisting of multiple phrases, separated by silence.
%   Each phrase consists of short pulses, separated by a slowly oscllating
%   inter-pulse-interval.
%
%   PARAMETERS
%
%   Defining pulse timing
%       meanIPI         mean interval between pulses (ms)
%       IPIOscPeriod    period of IPI oscillation (sec)
%       IPIOscAmp       amplitude of oscillation (proportion of mean, assume 10%)
%       IPIJitter       sd of random jitter added to each IPI (ms)
%       
%   Defining the component pulses themselves
%       pulseFreq       carrier frequency of pulse (Hz)
%       pulseDuration   duration of pulse (ms)
%       pulseRise       rise/fall time of pulse (ms), 3ms default
%
%   Defining overal phrase structure
%       songDuration    (s)
%       silenceDuration (s)
%       numPhrases      
%
%   Sound Synthesis
%       Fs              sampling rate of generated sound
%       filename        filename to save
%
%   OUTPUTS
%       snd             fly song
%       tSnd            timebase for song (e.g. plot(tSnd, snd))
%       pulse           component pulse of the song
%       tPulse          timebase for pulse (e.g. plot(tPulse, pulse)
%       tIPI            time of occurence for each pulse
%       IPIList         list of IPIs (plot(tIPI, IPIList))
%       idealIPIList    list of idealized IPIs (non-jittered)
%
%   JRI 12/22/04

%% PARAMETERS %%

filename = 'testsong.wav';

%test some basic values
meanIPI         = 35;   %msec
IPIOscPeriod    = 55;   %sec
IPIOscAmp       = 0.1;   % assumed: 10%
IPIJitter       = 0;    %try 0.25; %sd of pulse onset time jitter (in ms)

pulseFreq       = 220;  %Hz
pulseDuration   = 10;   %ms
pulseRise       = 3;    %ms

songDuration    = 3;    %seconds
silenceDuration = 2;    %seconds
soundDuration   = 60;   %total length of sound in seconds

Fs = 8000; %sample rate

%%%%%%%%%%%%%%%%%%
%% generate sound
%%%%%%%%%%%%%%%%%%

snd = zeros(1,round(soundDuration*Fs)); %start with silence

%synthesize basic pulse, shape its envelope
tPulse = 0:(1/Fs):(pulseDuration/1000);
pulse = sin(2*pi*pulseFreq*tPulse);
risePts = round(pulseRise/1000 * Fs); 
riseEnv = 1 - (cos(0.5 * pi * [0:(risePts-1)]/risePts) .^ 2);
fallEnv = riseEnv(end:-1:1);
pulse(1:risePts) = pulse(1:risePts) .* riseEnv;
pulse(end-risePts+1:end) = pulse(end-risePts+1:end) .* fallEnv;

figure
subplot(3,1,1)
plot(tPulse,pulse)
title('song pulse')

%now place pulses into sound, based on oscillating IPI
phraseDuration = (songDuration + silenceDuration);

t = 0;
phraseNum = 1;
%record IPI and pulse times
tIPI = [];
IPIList = [];
idealIPIList = [];

while (t < soundDuration - .2),
   %if we've reached the end of the song in current phrase, skip to next phrase
   if (t > songDuration + phraseDuration * (phraseNum -1)),
      t = t + silenceDuration;
      phraseNum = phraseNum + 1;
   end
   
   %place a pulse at current time
   idx = round(t * Fs) + 1;
   pulseIdx = idx:(idx+length(pulse)-1);
   snd(pulseIdx) = pulse;
   
   %get the idealized IPI (in ms) for this time in cycle
   idealIPI = meanIPI * ( 1 + IPIOscAmp * sin(2*pi*(1/IPIOscPeriod)*t) );
   %jitter this by a random amount
   IPI = idealIPI + randn(1,1)*IPIJitter;
   %record current pulse time, and current IPI
   tIPI = [tIPI t];
   IPIList = [IPIList IPI];
   idealIPIList = [idealIPIList idealIPI];
   %advance time to next pulse
   t = t + IPI/1000;
end

tSnd = (0:length(snd)-1) * (1/Fs);

subplot(3,1,2)
plot(tSnd, snd)
title('song audio')

subplot(3,1,3)
plot(tIPI,IPIList,'o')
plot(tIPI,idealIPIList,'r:')
title('IPI')

audiowrite(filename,snd,Fs)


