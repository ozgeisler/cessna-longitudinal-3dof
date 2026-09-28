clear
clc
close all

temp  = load('trim_60ms.mat');
XStar = temp.XStar;
UStar = temp.UStar;

tfinal = 30;
sim("cessna_doublet.slx")

t = simX.Time;
X = simX.Data;

% States
figure
labels = {'u','w','q','\theta','P_N','P_D'};
for k = 1:6
    subplot(3,2,k)
    plot(t,X(:,k),'LineWidth',1.2)
    ylabel(labels{k})
    xlabel('Time /s')
    grid on
end
sgtitle('Elevator Doublet - States')


