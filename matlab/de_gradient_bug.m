clear all, close all, clc



while(1)
    plot_rssi_array = dlmread('~/.ros/plot_rssi_array.txt');
    plot_rssi_heading_array = dlmread('~/.ros/plot_rssi_heading_array.txt');
    plot_angle_rssi = dlmread('~/.ros/plot_angle_rssi.txt');
    fid = fopen('~/.ros/rel_loc_x.txt');data = textscan(fid,'%f');rel_loc_x = data{1,1};
    fid = fopen('~/.ros/rel_loc_y.txt'); data = textscan(fid,'%f');rel_loc_y = data{1,1};
    fid = fopen('~/.ros/rel_x_per.txt');data = textscan(fid,'%f');rel_x_per = data{1,1};
    fid = fopen('~/.ros/rel_y_per.txt'); data = textscan(fid,'%f');rel_y_per = data{1,1};
    fid = fopen('~/.ros/rel_x.txt');data = textscan(fid,'%f');rel_x = data{1,1};
    fid = fopen('~/.ros/rel_y.txt'); data = textscan(fid,'%f');rel_y = data{1,1};    
    %figure(1),plot(rad2deg(plot_rssi_heading_array),plot_rssi_array)
    
    figure(1),subplot(1,2,1),polarplot(plot_rssi_heading_array+pi,plot_rssi_array)
    [value index1] = max(plot_rssi_array);
    hold on
    polarplot(plot_rssi_heading_array(index1),max(plot_rssi_array),'*r')
    polarplot(plot_angle_rssi(1)+pi,min(plot_rssi_array),'*g')
    ax = gca;
    ax.ThetaLim=[-180,180];
    ax.ThetaZeroLocation = 'bottom';
    hold off
    %disp(plot_rssi_heading_array(end) - plot_angle_rssi(1))
    %disp(plot_angle_rssi(2))
    title([num2str(plot_rssi_heading_array(end) - plot_angle_rssi(1)),' ',num2str(plot_angle_rssi(2))])
    
    if length(rel_loc_x)==length(rel_loc_y)
        
        subplot(1,2,2), hold on, plot(rel_loc_x,rel_loc_y,'b.'),
        if length(rel_x_per)==length(rel_y_per)
        plot(rel_x_per,rel_y_per,'g.')
        end
        if length(rel_x) == length(rel_y)
        plot(rel_x,rel_y,'r.')
        end
        hold off
    end
    pause(0.1)
end

