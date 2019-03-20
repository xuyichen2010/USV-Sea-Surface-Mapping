load('waveBack.mat');
load('waveTop.mat');
load('waveFront.mat');
load('sectorTop.mat');
load('sectorBack.mat');
load('sectorFront.mat');

figure(1);
indexBack = find(waveBack(:,3) > 0);
waveBackProcess = waveBack(indexBack,3);
meanWaveBack = mean(waveBackProcess);
medianWaveBack = median(waveBackProcess);
subplot(1,3,1);
histogram(waveBackProcess);
s = sprintf("Wave back (median: %.2f m/s, average: %.2f m/s \n",medianWaveBack, meanWaveBack);
title(s, 'FontSize', 16);


indexTop = find(waveTop(:,3) > 0);
waveTopProcess = waveTop(indexTop,3);
meanWaveTop = mean(waveTopProcess);
medianWaveTop = median(waveTopProcess);
subplot(1,3,2);
histogram(waveTopProcess);
s = sprintf("Wave top (median: %.2f m/s, average: %.2f m/s \n",medianWaveTop, meanWaveTop);
title(s, 'FontSize', 16);


indexFront = find(waveFront(:,3) > 0);
waveFrontProcess = waveBack(indexFront,3);
meanWaveFront = mean(waveFrontProcess);
medianWaveFront = median(waveFrontProcess);
subplot(1,3,3);
histogram(waveFrontProcess);
s = sprintf("Wave front (median: %.2f m/s, average: %.2f m/s \n",medianWaveFront, meanWaveFront);
title(s, 'FontSize', 16);

figure(2);
count = 1;
for i=1:8
    indexBack = find(back_height_tracker(i,:,4) > 0);
    waveBackProcess = back_height_tracker(i,indexBack,4);
    meanWaveBack = mean(waveBackProcess);
    medianWaveBack = median(waveBackProcess);
%     figure(count);
    subplot(4,6,count);
    count = count + 1;
    histogram(waveBackProcess);
    s = sprintf("Sector %d \n BACK M: %.2f m/s, A: %.2f m/s \n",i,medianWaveBack, meanWaveBack);
    title(s, 'FontSize', 12);


    indexTop = find(max_height_tracker(i,:,4) > 0);
    waveTopProcess = max_height_tracker(i,indexTop,4);
    meanWaveTop = mean(waveTopProcess);
    medianWaveTop = median(waveTopProcess);
    subplot(4,6,count);
    count = count + 1;
    histogram(waveTopProcess);
    s = sprintf("TOP M: %.2f m/s, A: %.2f m/s \n",medianWaveTop, meanWaveTop);
    title(s, 'FontSize', 12);


    indexFront = find(first_height_tracker(i,:,4) > 0);
    waveFrontProcess = first_height_tracker(i,indexFront,4);
    meanWaveFront = mean(waveFrontProcess);
    medianWaveFront = median(waveFrontProcess);
    subplot(4,6,count);
    count = count + 1;
    histogram(waveFrontProcess);
    s = sprintf("FRONT M: %.2f m/s, A: %.2f m/s \n",medianWaveFront, meanWaveFront);
    title(s, 'FontSize', 12);
    
end