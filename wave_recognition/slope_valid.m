load('slope.mat')

[m, n] = size(slope_direction);
x = 1:m;
x = x/15;
figure(1);
plot(x, slope_direction(:,2),'lineWidth',1);
hold on;
plot(x, slope_direction(:,1),'lineWidth',1);
hold off;

legend({'wave front','covariance'},'FontSize',16);
ylabel('slope', 'FontSize',16); 
xlabel('time(s)','FontSize',16);
s = sprintf('Two Direction Model\n');
title(s, 'FontSize', 16);