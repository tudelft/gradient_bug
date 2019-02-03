clear all;
close all;
clc;
    dirname='/home/knmcguire/Software/catkin_ws/src/bug_algorithms/statistics_scripts/experiments';
   
    %testname = strcat(dirname,'/test_20181014_231716');
   
    %testname = strcat(dirname,'/test_20181018_183757');   
   % testname = strcat(dirname,'/test_20180927_102815');

    %testname = strcat(dirname,'/test_20180926_192026');
    testname = strcat(dirname,'/test_20180926_233933');
    %testname = strcat(dirname,'/test_20190116_200134');

    made_it=[]

    %made_it_check = importdata(strcat(testname,'/made_it.txt'));
 
    
for(it=0:99)

    filename = strcat(testname,'/environment',num2str(it));
    
    img=imread(strcat(filename,'/environment.png'));
    trajectory = importdata(strcat(filename,'/trajectory.txt'));
    
    
    if size(trajectory,1)>30000
        made_it(it+1) = 0;
    else
        made_it(it+1) = 1;
    end
        
%     figure(1),
%     hold on,
%     xImg = linspace(-10, 10, size(img, 2));
%     yImg = linspace(-10,10, size(img, 1));
%     image(xImg, yImg, img, 'CDataMapping', 'scaled');
%     colormap(gray(256)),hold on,plot(trajectory(:,2),trajectory(:,1))
%     plot(4,4,'ro')
%     plot(-8,-8,'go')
%     keyboard
end
%made_it = importdata(strcat(testname,'/made_it.txt'));

success_rate = sum(made_it)/length(made_it)
