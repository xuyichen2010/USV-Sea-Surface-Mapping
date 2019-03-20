load('depthFrame.mat');

%% re-assign frame number
for i=1:length(f_to_d(:,1))
    f_to_d(i,1) = i;
end

%% preprocessing data
peak = find(f_to_d(:,2) > 0);
peak_index2 = [];
peak_index1 = [];
f1 = 0;
peak_index1(end+1) = peak(1);
for i = 2:length(peak)
    if peak(i)-peak(i-1) > 7
        peak_index2(end+1) = peak(i-1);
        peak_index1(end+1) = peak(i);
    end
end
peak_index2(end+1) = peak(end);

for i=1:length(peak_index2)
    index1 = peak_index1(i);
    index2 = peak_index2(i);
    peak_height = max(f_to_d(index1:index2,2));
    for j=index1:index2
        f_to_d(j,2) = peak_height;
    end
end


%% calculate the intervals
data = [];
peak = find(f_to_d(:,2) > 0);
for i = 2:length(peak)
    if peak(i)-peak(i-1) > 1
        data(end+1) = peak(i)-peak(i-1);
    end
end

%% change the frame number into seconds
for i=1:length(f_to_d(:,1))
    f_to_d(i,1) = f_to_d(i,1)/15;
end
for i=1:length(data)
    data(i) = data(i)/15;
end

%% draw the graph
figure(1);
plot(f_to_d(:,1),f_to_d(:,2),'lineWidth',1);
s = sprintf('Peak depth vs frame rate (above 0.8m)');
title(s, 'FontSize', 16);
ylabel('depth(m)', 'FontSize',16); 
xlabel('seconds','FontSize',16);

figure(2);
hist(data);
s = sprintf('Hisgrame of time intervals, median: %.2f s',median(data));
title(s, 'FontSize', 16);
ylabel('frequency', 'FontSize',16); 
xlabel('seconds','FontSize',16);