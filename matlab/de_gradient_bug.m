clear all, close all, clc



while(1)
 plot_rssi_array = dlmread('~/.ros/plot_rssi_array.txt');
 plot_rssi_heading_array = dlmread('~/.ros/plot_rssi_heading_array.txt');
 %figure(1),plot(rad2deg(plot_rssi_heading_array),plot_rssi_array)

 figure(1),polarplot(plot_rssi_heading_array+pi,plot_rssi_array)
 [value index1] = max(plot_rssi_array);
 hold on
    polarplot(plot_rssi_heading_array(index1),min(plot_rssi_array),'*r')
 ax = gca;
 ax.ThetaLim=[-180,180];
ax.ThetaZeroLocation = 'top';
hold off

 pause(0.2)
end

