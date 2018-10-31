clear all;
close all;
clc;
dirname='/home/knmcguire/Software/catkin_ws/src/bug_algorithms/statistics_scripts/experiments';
testname = strcat(dirname,'/test_20181024_191634');
%    testname = strcat(dirname,'/test_20181018_183757');
%testname = strcat(dirname,'/test_20181021_150510');
amount_of_agents = 1;
amount_of_environments = 99;

do_plot = false;

time_treshold = 3900:2500:28900;false
time_treshold = 28900;
size_threshold = size(time_treshold,2);
map = jet(size_threshold);
id_color_map = [1 0 0; 0 1 0; 0 0 1];

for itt = 1:size_threshold
    
    for(it=0:amount_of_environments)
        
        filename = strcat(testname,'/environment',num2str(it));
        
        img=255-imread(strcat(filename,'/environment.png'));
        
        if do_plot
            figure(1),
            hold on,
            xImg = linspace(-10, 10, size(img, 2));
            yImg = linspace(-10,10, size(img, 1));
            image(xImg, yImg, img, 'CDataMapping', 'scaled');
            colormap(gray(256))
        end
        
        for itn = 1:amount_of_agents
            %             fid = fopen(strcat(filename,'/trajectory',num2str(itn),'.txt'),'rt');
            %             datacell = textscan(fid, '%f, %f', 'HeaderLines',5,'CollectOutput');
            %             fclose(fid)
            %
            if(testname == strcat(dirname,'/test_20181018_183757'))
                trajectory = importdata(strcat(filename,'/trajectory.txt'),',');
                
            else
                trajectory = importdata(strcat(filename,'/trajectory',num2str(itn),'.txt'),',');
            end
            for itl=3099:size(trajectory,1)
                
                if sqrt((trajectory(itl,1)-8)^2+(trajectory(itl,2)-8)^2)<2
                    
                    trajectory(itl+1:size(trajectory,1),:)=[];
                    break
                    
                end
            end
            
            if size(trajectory,1)>time_treshold(itt)
                
                
                made_it(it+1,itn) = 0;
                
            else
                made_it(it+1,itn) = 1;
            end
            
            
            
            if do_plot
                figure(1),
                plot(trajectory(1:3099,2),trajectory(1:3099,1),':','Color',[id_color_map(itn,:)],'LineWidth',2)
                plot(trajectory(3099,2),trajectory(3099,1),'o','Color',id_color_map(itn,:),'MarkerSize',20)
                plot(trajectory(3099:end,2),trajectory(3099:end,1),'Color',id_color_map(itn,:),'LineWidth',2)
                
                
            end
            
        end
        made_it(it+1,:)
        
        if do_plot
            figure(1)
            plot(8,8,'m*')
            plot(8,8,'mo','MarkerSize',90)
            
            keyboard
            hold off
            
        end
    end
    
    sum_made_it(:,itt) = sum(made_it,2);
    %     figure(2),histogram(sum_made_it,'facecolor',map(1,:),'facealpha',.5,'edgecolor','none'), hold on
    %     keyboard
    
end



%%
figure(2)

sum_made_it_it = sum(sum_made_it,1)

bar(sum_made_it_it)
legendCell = cellstr(num2str(((time_treshold+100)/10)'))
set(gca,'xticklabel',legendCell)
xlabel('time threshold')
ylabel('percentage agent returned')


% histdata = [];false
% for it=1:size_threshold
%
%     histdata = [histdata;histcounts(sum_made_it(:,it),0:amount_of_agents+1,'Normalization', 'probability')];
% end
% bar(0:amount_of_agents,histdata')
% legendCell = cellstr(num2str(((time_treshold+100)/10)'))
% legend(legendCell)
%
% xlabel('# agents returned')
% ylabel('occurance percentage')


%made_it = importdata(strcat(testname,'/made_it.txt'));

% success_rate = sum(made_it)/length(made_it)

%% look at environment in detail
close all
check_environment_number = 16
filename = strcat(testname,'/environment',num2str(check_environment_number));
        img=255-imread(strcat(filename,'/environment.png'));

figure(3),
hold on,

xImg = linspace(-10, 10, size(img, 2));
yImg = linspace(-10,10, size(img, 1));
image(xImg, yImg, img, 'CDataMapping', 'scaled');
colormap(gray(256))


trajectory = importdata(strcat(filename,'/trajectory',num2str(1),'.txt'),',');


 for itl=3099:size(trajectory,1)
                
                if sqrt((trajectory(itl,1)-8)^2+(trajectory(itl,2)-8)^2)<2
                    
                    trajectory(itl+1:size(trajectory,1),:)=[];
                    break
                    
                end
 end
   plot(trajectory(1:3099,2),trajectory(1:3099,1),':','Color',[id_color_map(itn,:)],'LineWidth',2)
plot(trajectory(3099,2),trajectory(3099,1),'o','Color',id_color_map(itn,:),'MarkerSize',20)
plot(trajectory(3099:end,2),trajectory(3099:end,1),'Color',id_color_map(itn,:),'LineWidth',2)         

 plot(trajectory(3099,2),trajectory(3099,1),'o','Color',id_color_map(itn,:),'MarkerSize',20)
  plot(8,8,'m*')
% for it = 11:10:size(trajectory,1)
%     
%     if it<3099
%         plot(trajectory(it-10:it,2),trajectory(it-10:it,1),':','Color',[id_color_map(itn,:)],'LineWidth',2)
%     else
%         plot(trajectory(it-10:it,2),trajectory(it-10:it,1),'Color',id_color_map(itn,:),'LineWidth',2)   
%     end
%     
%         plot(trajectory(it-10,2),trajectory(it-10,1),'wo','MarkerSize',10)   
% 
%     plot(trajectory(it,2),trajectory(it,1),'ko','MarkerSize',10)   
% 
%     
%     pause(0.2)
%     
% end
