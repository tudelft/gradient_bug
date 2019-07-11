clear all;
close all;
clc;
dirname='/home/knmcguire/Software/catkin_ws/src/bug_algorithms/statistics_scripts/experiments';


filenames = {    '/test_20190711_105355'};
redo_analysis = true


if redo_analysis == true
    
    amount_of_agents_array = 6;
    amount_of_environments = 30;
    
    
    time_treshold = 9000 %9000;
    size_threshold = size(time_treshold,2);
    map = jet(size_threshold);
    id_color_map = [1 0 0; 0 1 0; 0 0 1; 0.5 0.5 0; 0 0.5 0.5; 0.3 0.3 0.3; 0.5 0 0; 0 0.5 0; 0 0 0.5];
    
    coverage = [];
    
    results = [];
    for ita = 1:size(amount_of_agents_array,2)
        amount_of_agents = amount_of_agents_array(ita);
        
        testname = strcat(dirname,filenames{ita}); % 2 agents
        
        for itt = 1:size_threshold
            
            for it=0:amount_of_environments
                occMap = robotics.BinaryOccupancyGrid(20,20,1);
                occMap_return = robotics.BinaryOccupancyGrid(20,20,1);
                
                filename = strcat(testname,'/environment',num2str(it));
                
                img=255-imread(strcat(filename,'/environment.png'));
                
                img_lines = importdata(strcat(filename,'/environment_lines.txt'),' ');
                img_new = 255 * ones(200,200, 'uint8');
                %                 figure(2),
                
                
                hold on
                
                img_new= insertShape(img_new,'Line',img_lines,'Color','black','LineWidth',5);
                
                
                hold off
                
                
                accessable_area_percentage =1;
                %calculate which part of the environment is accessable
                
                img_bin = im2bw(img_new);
                
                
                CC=bwconncomp(img_bin);
                
                linearInd = sub2ind([200,200],100,100);
                index = 0;
                for it_temp=1:size(CC.PixelIdxList,1)
                    if ismember(linearInd,CC.PixelIdxList{it_temp})
                        index=it_temp;
                        break
                    end
                end
                img_color = img;
                img_color(CC.PixelIdxList{index}) = 125;
                
                accessable_area_mat = zeros(200);
                accessable_area_mat(CC.PixelIdxList{index}) = 1;
                accessable_area_mat= imrotate(im2bw(imresize(accessable_area_mat,[20 20])),90);
                accessable_area_meter = numel(find(accessable_area_mat==1));
                
                %                 accessable_area_percentage =  numel(find(accessable_area_mat==1))/(20*20);
                accessable_area_percentage = (size(CC.PixelIdxList{index},1))/(200*200);
                %                 keyboard
                
                for itn = 1:amount_of_agents
                    
                    trajectory = importdata(strcat(filename,'/trajectory',num2str(itn),'.txt'),',');
                    
                    
                    for itl=(3000+itn*100-1):size(trajectory,1)
                        
                        if sqrt((trajectory(itl,1))^2+(trajectory(itl,2))^2)<2.5
                            
                            trajectory(itl+1:size(trajectory,1),:)=[];
                            break
                            
                        end
                        
                        
                        
                    end
                    
                    if size(trajectory,1)>time_treshold(itt)+itn*100
                        
                        made_it(it+1,itn) = 0;
                        trajectory(time_treshold(itt):end,:) = [];
                        
                    else
                        made_it(it+1,itn) = 1;
                    end
                    
                    
                    
                    
                    %coverage calculation
                    xy = trajectory + 10;
                    xy(end,:)=xy(end-1,:);
                    occMap_agent = robotics.BinaryOccupancyGrid(20,20,1);
                    setOccupancy(occMap_agent,xy,1);
                    OccMat_agent = occupancyMatrix(occMap_agent);
                    OccMat_agent = OccMat_agent.*accessable_area_mat;
                    OccMat_agent_all(:,:,itn) = OccMat_agent;
                    %                     coverage_per_agent(it+1,itn) = sum(OccMat_agent(:) == 1)/(numel(OccMat_agent)*accessable_area_percentage);
                    coverage_per_agent(it+1,itn) = sum(OccMat_agent(:) == 1)/(accessable_area_meter);
                    
                    
                    %
                    figure(5),show(occMap_agent),title(num2str(itn))
                    
                    setOccupancy(occMap,xy,1)
                    
                    if made_it(it+1,itn) == 1
                        setOccupancy(occMap_return,xy,1);
                    end
                    
                    
                    
                    
                end
                
                %check uniqueness coverage per agent
                %                 unique_coverage = zeros(size(OccMat_agent_all(:,:,1)));
                %                 for itn = 1:amount_of_agents
                %                     for itx = 1:size(OccMat_agent_all,1)
                %                         for ity = 1:size(OccMat_agent_all,2)
                %
                %                             OccMat_agent_array = OccMat_agent_all(itx,ity,:);
                %                             if  OccMat_agent_all(itx,ity,itn) == true && numel(find(OccMat_agent_array==true))==1
                %                                 unique_coverage(itx,ity) =1;
                %                             else
                %                                 unique_coverage(itx,ity) =0;
                %
                %                             end
                %
                %                         end
                %                     end
                %                     unique_coverage_per_agent(itn) = sum(unique_coverage(:)==1)/numel(unique_coverage);
                %                 end
                unique_coverage_per_agent_tot(it+1) = 0;%mean(unique_coverage_per_agent);
                
                occMat = occupancyMatrix(occMap);
                occMat_return = occupancyMatrix(occMap_return);
                
                occMat=occMat.*accessable_area_mat;
                occMat_return=occMat_return.*accessable_area_mat;
                
                %                 coverage_tot_return(it+1,1) = sum(occMat(:) == 1)/(numel(occMat)*accessable_area_percentage);
                %                 coverage_tot_return(it+1,2) = sum(occMat_return(:) == 1)/(numel(occMat_return)*accessable_area_percentage);
                coverage_tot_return(it+1,1) = sum(occMat(:) == 1)/(accessable_area_meter);
                coverage_tot_return(it+1,2) = sum(occMat_return(:) == 1)/(accessable_area_meter);
                
                
%                 figure(1),%subplot(2,2,1),
%                 hold on,
%                 
%                 xImg = linspace(-10, 10, size(img_new, 2));
%                 yImg = linspace(-10,10, size(img_new, 1));
%                 image(xImg, yImg, img_new, 'CDataMapping', 'scaled');
%                 colormap(gray(256))
%                 hold on
%                 plot(trajectory(:,2),trajectory(:,1))
%                 keyboard
%                 
                
                %                 figure(1),   subplot(2,1,1),
                %                 imshow(imrotate(img_color,90));
                %                 imshow(imrotate(accessable_area_mat,90))
                %                 figure(1),subplot(2,1,2)
                %                 C = imfuse(occMat,imrotate(accessable_area_mat,90),'falsecolor','Scaling','joint','ColorChannels',[1 2 0]);
                %
                %                 imshow(C)
                %                 sum(occMat(:) == 1)/(numel(occMat))
                %                 sum(occMat(:) == 1)/(numel(occMat)*accessable_area_percentage)
                %
                %
                %                 keyboard
                %
                
                
                made_it(it+1,:)
                
                
                
            end
            
            
            sum_made_it(:,itt) = sum(made_it,2);
            %          figure(2),histogram(sum_made_it,'facecolor',map(1,:),'facealpha',.5,'edgecolor','none'), hold on
            
            
        end
        
        results(ita).made_it = made_it;
        
        results(ita).sum_made_it = sum_made_it;
        results(ita).coverage_tot_return = coverage_tot_return;
        results(ita).coverage_per_agent = coverage_per_agent;
        
    end
    %     save('simulation_results.mat')
    
else
    load('simulation_results.mat')
    
end
keyboard

%%
close all



for ita = 1:size(amount_of_agents_array,2)
    amount_of_agents = amount_of_agents_array(ita);
    sum_made_it= results(ita).sum_made_it;
    made_it_rate(:,ita) = sum_made_it/amount_of_agents;
    
    made_it_mean(ita) = mean(made_it_rate(:,ita));
    made_it_std(ita) = std(made_it_rate(:,ita));
    
    coverage_tot_return = results(ita).coverage_tot_return;
    coverage_tot_mean(ita) = mean(coverage_tot_return(:,1));
    coverage_tot_std(ita) = std(coverage_tot_return(:,1));
    coverage_return_mean(ita) = mean(coverage_tot_return(:,2));
    coverage_return_std(ita) = std(coverage_tot_return(:,2));
    
    
    made_it =results(ita).made_it;
    %     coverage_per_agent = results(ita).coverage_per_agent.*made_it;
    %     mean_per_row = sum(coverage_per_agent,2)./sum_made_it;
    %     mean_per_row(isnan(mean_per_row))=0;
    
    coverage_per_agent = results(ita).coverage_per_agent;
    mean_per_row_per_agent(:,ita) = mean(coverage_per_agent,2);
    
    coverage_per_agent_mean(ita) = mean(mean_per_row_per_agent(:,ita));
    coverage_per_agent_std(ita) = std(mean_per_row_per_agent(:,ita));
    
    
end

%plot return rate%%
figure(2),
bar(made_it_mean,'grouped','FaceColor',[0 1 1],'EdgeColor',[0 0 0])
set(gca,'xticklabel',{'2','4','6','8','10'})

hold on,
errorbar(made_it_mean,made_it_std,'Color',[0 0 0],'LineWidth',3)
ylabel('return rate')
ylim([0 1.2])

%plot coverage%%
figure(3)
coverage_bar_mean_array = [coverage_tot_mean; coverage_return_mean; coverage_per_agent_mean]';
coverage_bar_std_array = [coverage_tot_std; coverage_return_std; coverage_per_agent_std]';

bar(coverage_bar_mean_array,'grouped')
hold on
%errorbar on grouped bar chart
ngroups = size(coverage_bar_mean_array, 1);
nbars = size(coverage_bar_mean_array, 2);
groupwidth = min(0.8, nbars/(nbars + 1.5));
for i = 1:nbars
    x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
    errorbar(x, coverage_bar_mean_array(:,i), coverage_bar_std_array(:,i), 'k','LineWidth',2);
end
ylim([0 1.2])
set(gca,'xticklabel',{'2','4','6','8','10'})

legend('Coverage total', 'Coverage returned', 'Coverage per agent')

keyboard

%% look at environment in detail
close all
experiment = 1;
testname = strcat(dirname,filenames{1}); % 2 agents
amount_of_agents = amount_of_agents_array(experiment)
check_environment_number =20;


filename = strcat(testname,'/environment',num2str(check_environment_number));
img=255-imread(strcat(filename,'/environment.png'));
figure(3),
hold on,

xImg = linspace(-10, 10, size(img, 2));
yImg = linspace(-10,10, size(img, 1));
image(xImg, yImg, img, 'CDataMapping', 'scaled');
colormap(gray(256))

for itn=1:amount_of_agents
    
    trajectory = importdata(strcat(filename,'/trajectory',num2str(itn),'.txt'),',');
    plot(trajectory(1:3099,2),trajectory(1:3099,1),':','Color',[id_color_map(itn,:)],'LineWidth',2)
    plot(trajectory(3099,2),trajectory(3099,1),'o','Color',id_color_map(itn,:),'MarkerSize',20)
    plot(trajectory(3099:end,2),trajectory(3099:end,1),'Color',id_color_map(itn,:),'LineWidth',2)
    
    plot(trajectory(3099,2),trajectory(3099,1),'o','Color',id_color_map(itn,:),'MarkerSize',20)
    plot(0,0,'m*')
    
end


%% Statistical analysis

close all
% coverage total vs number of drones

y= [results(1).coverage_tot_return(:,1);results(2).coverage_tot_return(:,1);results(3).coverage_tot_return(:,1);results(4).coverage_tot_return(:,1);results(5).coverage_tot_return(:,1)];
x=[ones(1,100)*2,ones(1,100)*4,ones(1,100)*6,ones(1,100)*8, ones(1,100)*10]';


mdl=fitlm(x,y,'quadratic')
mdl.Rsquared.Ordinary
yCalc1 = mdl.Coefficients{3,1}*x.^2+mdl.Coefficients{2,1}*x+mdl.Coefficients{1,1};


figure,
scatter(x,y)
hold on
plot(x,yCalc1)


% coverage returned vs number of drones

y= [results(1).coverage_tot_return(:,2);results(2).coverage_tot_return(:,2);results(3).coverage_tot_return(:,2);results(4).coverage_tot_return(:,2);results(5).coverage_tot_return(:,2)];


mdl=fitlm(x,y,'quadratic')
mdl.Rsquared.Ordinary
yCalc1 =mdl.Coefficients{3,1}*x.^2 +mdl.Coefficients{2,1}*x+mdl.Coefficients{1,1};

figure,
scatter(x,y)
hold on
plot(x,yCalc1)

% coverage returned vs number of drones

y= [mean_per_row_per_agent(:,1);mean_per_row_per_agent(:,2);mean_per_row_per_agent(:,3);mean_per_row_per_agent(:,4);mean_per_row_per_agent(:,5)];


mdl=fitlm(x,y,'quadratic')
mdl.Rsquared.Ordinary
yCalc1 =mdl.Coefficients{3,1}*x.^2 +mdl.Coefficients{2,1}*x+mdl.Coefficients{1,1};

figure,
scatter(x,y)
hold on
plot(x,yCalc1)

% return rate vs number of drones

y = [made_it_rate(:,1);made_it_rate(:,2);made_it_rate(:,3);made_it_rate(:,4);made_it_rate(:,5)];

mdl=fitlm(x,y,'quadratic')
mdl.Rsquared.Ordinary
yCalc1 = mdl.Coefficients{3,1}*x.^2+mdl.Coefficients{2,1}*x+mdl.Coefficients{1,1};


figure,
scatter(x,y)
hold on
plot(x,yCalc1)



