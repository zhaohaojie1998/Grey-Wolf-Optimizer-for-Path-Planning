%%%   灰狼优化算法航迹规划   %%%
clc, close all


%--- 算法选择 1：GWO算法  2：MP-GWO算法
options = 2;


%--- 算法参数设置          
SearchAgents = 60;           % 搜索智能体个数/狼群数量/可行解的个数 （>= 20）
Max_iter = 145 ;             % 最大搜索步数     


%--- 协同无人机设置
UAV = UAV_SetUp;             % 在 UAV_SetUp.m 文件进行设置


%--- 灰狼算法
if options < 2
    solution = GWO(UAV, SearchAgents, Max_iter);  % GWO算法
else
    solution = MP_GWO(UAV, SearchAgents, Max_iter);  % MP-GWO算法
end


%--- 绘图
IMG_AutoPlot(solution, UAV) % 自适应绘图（全自动绘图，但不如手动的好看）
% IMG_Plot(solution, UAV) % 手动绘图（需手动添加无人机，默认3个）



