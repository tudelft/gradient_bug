clear all, close all, clc

figure(1),subplot(1,2,2),plot(12,12,'bo'),hold on
ha1=plot(0,0,'b.');
ha2=plot(0,0,'g.');
ha3=plot(0,0,'r.');

xlim([-2 18])
ylim([-2 18])

rel_loc_x_array=[];
rel_loc_y_array=[];
rel_x_array=[];
rel_y_array=[];
rel_x_per_array=[];
rel_y_per_array=[];


while(1)
    plot_rssi_array = dlmread('~/.ros/plot_rssi_array.txt');
    plot_rssi_heading_array = dlmread('~/.ros/plot_rssi_heading_array.txt');
    plot_angle_rssi = dlmread('~/.ros/plot_angle_rssi.txt');
    %Real odoometry
    fid = fopen('~/.ros/rel_loc_x.txt');data = textscan(fid,'%f');rel_loc_x = data{1,1};
    fid = fopen('~/.ros/rel_loc_y.txt'); data = textscan(fid,'%f');rel_loc_y = data{1,1};
    %Perfect odometry
    fid = fopen('~/.ros/rel_x_per.txt');data = textscan(fid,'%f');rel_x_per = data{1,1};
    fid = fopen('~/.ros/rel_y_per.txt'); data = textscan(fid,'%f');rel_y_per = data{1,1};
    %adapted odometry
    fid = fopen('~/.ros/rel_x.txt');data = textscan(fid,'%f');rel_x = data{1,1};
    fid = fopen('~/.ros/rel_y.txt'); data = textscan(fid,'%f');rel_y = data{1,1};    
    %figure(1),plot(rad2deg(plot_rssi_heading_array),plot_rssi_array)
    
    %figure(1),subplot(1,2,1),polarplot(plot_rssi_heading_array+pi,plot_rssi_array)
    [value index1] = max(plot_rssi_array);
  %  hold on
    %polarplot(plot_rssi_heading_array(index1),max(plot_rssi_array),'*r')
   % polarplot(plot_angle_rssi(1)+pi,min(plot_rssi_array),'*g')
  %  ax = gca;
   % ax.ThetaLim=[-180,180];
   % ax.ThetaZeroLocation = 'bottom';
   % hold off
    %disp(plot_rssi_heading_array(end) - plot_angle_rssi(1))
    %disp(plot_angle_rssi(2))
%    title([num2str(plot_rssi_heading_array(end) - plot_angle_rssi(1)),' ',num2str(plot_angle_rssi(2))])
            
        %subplot(1,2,2), hold on, plot(12,12,'bo'), 
        if ~isempty(rel_loc_x) &&  ~isempty(rel_loc_y)
        rel_loc_x_array = [rel_loc_x_array;rel_loc_x];
        rel_loc_y_array = [rel_loc_y_array;rel_loc_y];
        end
        if ~isempty(rel_x) && ~isempty(rel_y)
        rel_x_array = [rel_x_array;rel_x];
        rel_y_array = [rel_y_array;rel_y];
        end
        if ~isempty(rel_x_per) && ~isempty(rel_y_per)
        rel_x_per_array = [rel_x_per_array;rel_x_per];
        rel_y_per_array = [rel_y_per_array;rel_y_per];
        end
        set(ha1,'XData',rel_loc_x_array,'YData',rel_loc_y_array);
         set(ha2,'XData',rel_x_per_array,'YData',rel_y_per_array);
        set(ha3,'XData',rel_x_array,'YData',rel_y_array);
        drawnow
        pause(0.1)
        
       % hold off
    %end
   % pause(0.1)
end

