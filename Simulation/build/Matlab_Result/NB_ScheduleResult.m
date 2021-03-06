clear;
clc;
%=============模擬結果===========%
%% 檔案宣告 & 讀檔
%A(1,:) 第一Row ; A（：，1）第一Col

%% Scenario 1 Sort1/2 
AverageDelay_Sort1_payloadSizeCE0 = csvread('AverageDelay_Sort1_payloadSizeCE0.csv');
AverageDelay_Sort1_payloadSizeCE1 = csvread('AverageDelay_Sort1_payloadSizeCE1.csv');
AverageDelay_Sort1_payloadSizeCE2 = csvread('AverageDelay_Sort1_payloadSizeCE2.csv');
AverageDelay_Sort2_payloadSizeCE0 = csvread('AverageDelay_Sort2_payloadSizeCE0.csv');
AverageDelay_Sort2_payloadSizeCE1 = csvread('AverageDelay_Sort2_payloadSizeCE1.csv');
AverageDelay_Sort2_payloadSizeCE2 = csvread('AverageDelay_Sort2_payloadSizeCE2.csv');

resourceUtili_Sort1_payloadSize = csvread('resourceUtili_Sort1_payloadSize.csv');
resourceUtili_Sort2_payloadSize = csvread('resourceUtili_Sort2_payloadSize.csv');
%% Scenario 2 High/Low PP setting 
AverageDelay_highPP_payloadSizeCE0 = csvread('AverageDelay_highPP_payloadSizeCE0.csv');
AverageDelay_highPP_payloadSizeCE1 = csvread('AverageDelay_highPP_payloadSizeCE1.csv');
AverageDelay_highPP_payloadSizeCE2 = csvread('AverageDelay_highPP_payloadSizeCE2.csv');
AverageDelay_lowPP_payloadSizeCE0 = csvread('AverageDelay_lowPP_payloadSizeCE0.csv');
AverageDelay_lowPP_payloadSizeCE1 = csvread('AverageDelay_lowPP_payloadSizeCE1.csv');
AverageDelay_lowPP_payloadSizeCE2 = csvread('AverageDelay_lowPP_payloadSizeCE2.csv');

resourceUtili_highPP_payloadSize = csvread('resourceUtili_highPP_payloadSize.csv');
resourceUtili_lowPP_payloadSize = csvread('resourceUtili_lowPP_payloadSize.csv');

%% Scenario 3 Diff Bufer size mapping method
AverageDelay_Highest_payloadSizeCE0 = csvread('AverageDelay_Highest_payloadSizeCE0.csv');
AverageDelay_Highest_payloadSizeCE1 = csvread('AverageDelay_Highest_payloadSizeCE1.csv');
AverageDelay_Highest_payloadSizeCE2 = csvread('AverageDelay_Highest_payloadSizeCE2.csv');
AverageDelay_Medium_payloadSizeCE0 = csvread('AverageDelay_Medium_payloadSizeCE0.csv');
AverageDelay_Medium_payloadSizeCE1 = csvread('AverageDelay_Medium_payloadSizeCE1.csv');
AverageDelay_Medium_payloadSizeCE2 = csvread('AverageDelay_Medium_payloadSizeCE2.csv');
AverageDelay_Lowest_payloadSizeCE0 = csvread('AverageDelay_Lowest_payloadSizeCE0.csv');
AverageDelay_Lowest_payloadSizeCE1 = csvread('AverageDelay_Lowest_payloadSizeCE1.csv');
AverageDelay_Lowest_payloadSizeCE2 = csvread('AverageDelay_Lowest_payloadSizeCE2.csv');

resourceUtili_Highest_payloadSize = csvread('resourceUtili_Highest_payloadSize.csv');
resourceUtili_Medium_payloadSize = csvread('resourceUtili_Medium_payloadSize.csv');
resourceUtili_Lowest_payloadSize = csvread('resourceUtili_Lowest_payloadSize.csv');

%% Scenario 3 Low-Complexity DL subframe
timeCost_bitmap = csvread('timeCost_bitmap.csv');
timeCost = csvread('timeCost.csv');
%% Scenario 4 NPUSCH decent way
AverageDelay_NPUSCH1_payloadSizeCE0 = csvread('AverageDelay_NPUSCH1_payloadSizeCE0.csv');
AverageDelay_NPUSCH1_payloadSizeCE1 = csvread('AverageDelay_NPUSCH1_payloadSizeCE1.csv');
AverageDelay_NPUSCH1_payloadSizeCE2 = csvread('AverageDelay_NPUSCH1_payloadSizeCE2.csv');
AverageDelay_NPUSCH2_payloadSizeCE0 = csvread('AverageDelay_NPUSCH2_payloadSizeCE0.csv');
AverageDelay_NPUSCH2_payloadSizeCE1 = csvread('AverageDelay_NPUSCH2_payloadSizeCE1.csv');
AverageDelay_NPUSCH2_payloadSizeCE2 = csvread('AverageDelay_NPUSCH2_payloadSizeCE2.csv');

resourceUtili_NPUSCH1_payloadSize = csvread('resourceUtili_NPUSCH1_payloadSize.csv');
resourceUtili_NPUSCH2_payloadSize = csvread('resourceUtili_NPUSCH2_payloadSize.csv');

%% Scenario 1 Sort1/2 
ys_Sort1_Average=AverageDelay_Sort1_payloadSizeCE0(:,3);
ys_Sort2_Average=AverageDelay_Sort2_payloadSizeCE0(:,3);

xs_Sort1_0 = AverageDelay_Sort1_payloadSizeCE0(:,1);
ys_Sort1_0 = AverageDelay_Sort1_payloadSizeCE0(:,2);
ys_Sort1_1 = AverageDelay_Sort1_payloadSizeCE1(:,2);
ys_Sort1_2 = AverageDelay_Sort1_payloadSizeCE2(:,2);

ys_Sort2_0 = AverageDelay_Sort2_payloadSizeCE0(:,2);
ys_Sort2_1 = AverageDelay_Sort2_payloadSizeCE1(:,2);
ys_Sort2_2 = AverageDelay_Sort2_payloadSizeCE2(:,2);

ys_Sort1_U = resourceUtili_Sort1_payloadSize(:,2);
ys_Sort2_U = resourceUtili_Sort2_payloadSize(:,2);

%% Scenario 2 High/Low PP setting 
xs_PP_0 = AverageDelay_highPP_payloadSizeCE0(:,1);
ys_HPP_0 = AverageDelay_highPP_payloadSizeCE0(:,2);
ys_HPP_1 = AverageDelay_highPP_payloadSizeCE1(:,2);
ys_HPP_2 = AverageDelay_highPP_payloadSizeCE2(:,2);

ys_LPP_0 = AverageDelay_lowPP_payloadSizeCE0(:,2);
ys_LPP_1 = AverageDelay_lowPP_payloadSizeCE1(:,2);
ys_LPP_2 = AverageDelay_lowPP_payloadSizeCE2(:,2);

ys_HPP_Aver = AverageDelay_highPP_payloadSizeCE0(:,3);
ys_LPP_Aver = AverageDelay_lowPP_payloadSizeCE0(:,3);

xs_HPP_U = resourceUtili_highPP_payloadSize(:,1);
ys_HPP_U = resourceUtili_highPP_payloadSize(:,2);
ys_LPP_U = resourceUtili_lowPP_payloadSize(:,2);
%% Scenario 3 Diff Bufer size mapping method
xs_H_0 = AverageDelay_Highest_payloadSizeCE0(:,1);
ys_H_0 = AverageDelay_Highest_payloadSizeCE0(:,2);
ys_H_1 = AverageDelay_Highest_payloadSizeCE1(:,2);
ys_H_2 = AverageDelay_Highest_payloadSizeCE2(:,2);

ys_M_0 = AverageDelay_Medium_payloadSizeCE0(:,2);
ys_M_1 = AverageDelay_Medium_payloadSizeCE1(:,2);
ys_M_2 = AverageDelay_Medium_payloadSizeCE2(:,2);

ys_L_0 = AverageDelay_Lowest_payloadSizeCE0(:,2);
ys_L_1 = AverageDelay_Lowest_payloadSizeCE1(:,2);
ys_L_2 = AverageDelay_Lowest_payloadSizeCE2(:,2);

ys_H_U = resourceUtili_Highest_payloadSize(:,2);
ys_M_U = resourceUtili_Medium_payloadSize(:,2);
ys_L_U = resourceUtili_Lowest_payloadSize(:,2);

%% Scenario 3 Low-Complexity DL subframe
xs_T_0 = timeCost_bitmap(:,1);
ys_T_0 = timeCost_bitmap(:,2);
ys_T_1 = timeCost(:,2);

%% Scenario 4 NPUSCH decent way
ys_NPUSCH1_Average=AverageDelay_NPUSCH1_payloadSizeCE0(:,3);
ys_NPUSCH2_Average=AverageDelay_NPUSCH2_payloadSizeCE0(:,3);

xs_NPUSCH1_0 = AverageDelay_NPUSCH1_payloadSizeCE0(:,1);
ys_NPUSCH1_0 = AverageDelay_NPUSCH1_payloadSizeCE0(:,2);
ys_NPUSCH1_1 = AverageDelay_NPUSCH1_payloadSizeCE1(:,2);
ys_NPUSCH1_2 = AverageDelay_NPUSCH1_payloadSizeCE2(:,2);

ys_NPUSCH2_0 = AverageDelay_NPUSCH2_payloadSizeCE0(:,2);
ys_NPUSCH2_1 = AverageDelay_NPUSCH2_payloadSizeCE1(:,2);
ys_NPUSCH2_2 = AverageDelay_NPUSCH2_payloadSizeCE2(:,2);

ys_NPUSCH1_U = resourceUtili_NPUSCH1_payloadSize(:,2);
ys_NPUSCH2_U = resourceUtili_NPUSCH2_payloadSize(:,2);
%% Scenario 1 Sort1/2 
figure(1)
hs = plot(xs_Sort1_0, ys_Sort1_Average,'bo-',xs_Sort1_0,ys_Sort2_Average,'rsquare-');
legend('Sorting 1:CE0/1/2','Sorting 2:CE0/1/2');
set(hs, 'linewidth', 1.0);    % 曲線寬度改為1.5
axis([0, 200, 0,2000]);
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE  for three CE levels [M=60][Ma=20/sec]');
grid on
figure(2)
hs1 = plot(xs_Sort1_0, ys_Sort1_0,'bo-',xs_Sort1_0,ys_Sort1_1,'rsquare-',xs_Sort1_0,ys_Sort1_2,'gdiamond-',xs_Sort1_0, ys_Sort2_0,'co-',xs_Sort1_0,ys_Sort2_1,'ksquare-',xs_Sort1_0,ys_Sort2_2,'mdiamond-');
legend('Sorting 1:CE0','Sorting 1:CE1','Sorting 1:CE2','Sorting 2:CE0','Sorting 2:CE1','Sorting 2:CE2');
set(hs1, 'linewidth', 1.0);    % 曲線寬度改為1.5
axis([0, 200, 0,4000]);
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE for three CE levels [M=60][Ma=20/sec]');
grid on
figure(3)
hs2 = plot(xs_Sort1_0, ys_Sort1_U,'bo-',xs_Sort1_0,ys_Sort2_U,'rsquare-');
legend('Sorting 1:CE0/1/2','Sorting 2:CE0/1/2');
set(hs2, 'linewidth', 1.0);    % 曲線寬度改為1.5
axis([0, 200, 0,1]);
xlabel('Payload Size of each UE'),ylabel('Resource utilization');
title('Resource utilization [M=60][Ma=20/sec]');
grid on
%% Scenario 2 High/Low PP setting 
figure(4)
hs3 = plot(xs_PP_0, ys_HPP_0,'bo-',xs_PP_0, ys_LPP_0,'co-');
legend('Case1:CE0','Case2:CE0');
set(hs3, 'linewidth', 1.0);    % 曲線寬度改為1.5
axis([0, 200, 0,600]);
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE for CE level 0 [M=60][Ma=20/sec]');
grid on
figure(5)
hs4 = plot(xs_PP_0,ys_HPP_1,'rsquare-',xs_PP_0,ys_LPP_1,'ksquare-');
legend('Case1:CE1','Case2:CE1');
set(hs4, 'linewidth', 1.0);    % 曲線寬度改為1.5
axis([0, 200, 0,2000]);
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE for CE level 1 [M=60][Ma=20/sec]');
grid on
figure(6)
hs5 = plot(xs_PP_0,ys_HPP_2,'gdiamond-',xs_PP_0,ys_LPP_2,'mdiamond-');
legend('Case1:CE2','Case2:CE2');
set(hs5, 'linewidth', 1.0);    % 曲線寬度改為1.5
axis([0, 200, 0,4000]);
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE for CE level 2 [M=60][Ma=20/sec]');
grid on
figure(7)
hs6 = plot(xs_PP_0, ys_HPP_0,'bo-',xs_PP_0,ys_HPP_1,'rsquare-',xs_PP_0,ys_HPP_2,'gdiamond-',xs_PP_0, ys_LPP_0,'co-',xs_PP_0,ys_LPP_1,'ksquare-',xs_PP_0,ys_LPP_2,'mdiamond-');
legend('Case1:CE0','Case1:CE1','Case1:CE2','Case2:CE0','Case2:CE1','Case2:CE2');
set(hs6, 'linewidth', 1.0);    % 曲線寬度改為1.5
axis([0, 200, 0,4000]);
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE  for three CE levels [M=30][Ma=6/sec]');
grid on
figure(13)
hs12 = plot(xs_PP_0, ys_HPP_Aver,'bo-',xs_PP_0,ys_LPP_Aver,'rsquare-');
legend('Case1','Case2');
set(hs12, 'linewidth', 1.0);    % 曲線寬度改為1.5
axis([0, 200, 0,2000]);
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE  for three CE levels [M=60][Ma=20/sec]');
grid on
figure(8)
hs7 = plot(xs_PP_0, ys_HPP_U,'bo-',xs_PP_0,ys_LPP_U,'rsquare-');
set(hs7, 'linewidth', 1.0);    % 曲線寬度改為1.5
axis([0, 200, 0,1]);
xlabel('Payload Size of each UE'),ylabel('Resource utilization');
legend('Case1','Case2');
title('Resource utilization [M=60][Ma=20/sec]');
grid on
%% Scenario 3 Diff Bufer size mapping method
figure(9)
hs8 = plot(xs_H_0, ys_H_0,'bo-',xs_H_0,ys_M_0,'rsquare-',xs_H_0,ys_L_0,'gdiamond-');
axis([0, 200, 0,600]);
legend('#1:CE0','#2:CE0','#3:CE0');
set(hs8, 'linewidth', 1.0);    % 曲線寬度改為1.5
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE  for three CE level 0');
grid on
figure(10)
hs9 = plot(xs_H_0, ys_H_1,'bo-',xs_H_0,ys_M_1,'rsquare-',xs_H_0,ys_L_1,'gdiamond-');
axis([0, 200, 0,600]);
legend('#1:CE1','#2:CE1','#3:CE1');
set(hs9, 'linewidth', 1.0);    % 曲線寬度改為1.5
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE  for three CE level 1');
grid on
figure(11)
hs10 = plot(xs_H_0, ys_H_2,'bo-',xs_H_0,ys_M_2,'rsquare-',xs_H_0,ys_L_2,'gdiamond-');
axis([0, 200, 0,800]);
legend('#1:CE2','#2:CE2','#3:CE2');
set(hs10, 'linewidth', 1.0);    % 曲線寬度改為1.5
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE  for three CE level 2');
grid on
figure(12)
hs11 = plot(xs_H_0, ys_H_U,'co-',xs_H_0,ys_M_U,'ksquare-',xs_H_0,ys_L_U,'ydiamond-');
set(hs11, 'linewidth', 1.0);    % 曲線寬度改為1.5
axis([0, 200, 0,1]);
xlabel('Payload Size of each UE'),ylabel('Resource utilization');
title('Resource utilization');
legend('#1','#2','#3');
grid on
%% Scenario 3 Low-Complexity DL subframe
figure(14)
% YT = zeros(1,11);
% YT(1)=0;
% for i = 1:11
% 	YT(i) = ys_T_0(i)-ys_T_1(i)/xs_T_0(i);
%     YT(i) = ys_T_0(i)-ys_T_1(i);
% end
% hs13 = plot(xs_T_0, ys_T_0,'bo-',xs_T_0,ys_T_1,'rsquare-',xs_T_0,YT,'gdiamond-');
hs13 = plot(xs_T_0, ys_T_0,'bo-',xs_T_0,ys_T_1,'rsquare-');
axis([0, 10000, 0,25]);
legend('With bitmap','Without bitmap','Time difference');
% legend('With bitmap','Without bitmap');
set(hs13, 'linewidth', 1.0);    % 曲線寬度改為1.5
xlabel('Simulation times'),ylabel('Processing time(s)');
title('Time cost with/without bitmap in DL subfrmae determination');
grid on

%% Scenario 4 NPUSCH decent way
figure(15)
hs14 = plot(xs_NPUSCH1_0, ys_NPUSCH1_0,'bo-',xs_NPUSCH1_0,ys_NPUSCH1_1,'rsquare-',xs_NPUSCH1_0,ys_NPUSCH1_2,'gdiamond-',xs_NPUSCH1_0, ys_NPUSCH2_0,'co-',xs_NPUSCH1_0,ys_NPUSCH2_1,'ksquare-',xs_NPUSCH1_0,ys_NPUSCH2_2,'mdiamond-');
axis([0, 200, 0,2000]);
legend('Method1:CE0','Method1:CE1','Method1:CE2','Method2:CE0','Method2:CE1','Method2:CE2');
set(hs14, 'linewidth', 1.0);    % 曲線寬度改為1.5
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE  for three CE level 0');
grid on
figure(16)
hs15 = plot(xs_NPUSCH1_0, ys_NPUSCH1_Average,'bo-',xs_NPUSCH1_0,ys_NPUSCH2_Average,'rsquare-');
axis([0, 200, 0,2000]);
legend('scheduling delay -based order','subcarrier indication -based order');
set(hs15, 'linewidth', 1.0);    % 曲線寬度改為1.5
xlabel('Payload Size of each UE'),ylabel('Average Delay (ms)');
title('Average Delay of each UE  for three CE level 0');
grid on
figure(17)
hs16 = plot(xs_NPUSCH1_0, ys_NPUSCH1_U,'bo-',xs_NPUSCH1_0,ys_NPUSCH2_U,'rsquare-');
set(hs11, 'linewidth', 0.5);    % 曲線寬度改為1.5
axis([0, 200, 0,0.5]);
xlabel('Payload Size of each UE'),ylabel('Resource utilization');
title('Resource utilization');
legend('scheduling delay -based order','subcarrier indication -based order');
grid on
