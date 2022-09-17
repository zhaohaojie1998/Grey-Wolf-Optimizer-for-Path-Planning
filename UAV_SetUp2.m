function [UAV] = UAV_SetUp2
%UAV_SETUP2 在此设置多无人机协同航迹规划任务
% 论文1的环境


% 航迹点设置
% （每行为一个无人机的参数）
UAV.S = [  0,           0;
                  0,           100;
                  300,       0;      ];     % 起点位置 (x,y)或(x,y,z)

UAV.G = [  875,      875;
                  800,       875;
                  875,       800;  ];      % 目标位置 (x,y)或(x,y,z)

UAV.PointNum = [  26;
                                  24;
                                  24;  ];                 % 每个无人机导航点个数（起始点之间点的个数）

UAV.PointDim = size(UAV.S, 2);        % 坐标点维度 （由 起点 坐标决定）
UAV.num = size(UAV.S, 1);                 % UAV数量 （由 起点 个数决定）


% 威胁点设置 (x,y,r) 或 (x,y,z,r)
% （每行为一个威胁的坐标和半径）
UAV.Menace.radar = [  200, 200,    20;
                                        600, 700,    20;  ];  % 雷达威胁（数学模型和其余威胁不同）

UAV.Menace.other = [  80, 40,   40;
                                        300, 300,   40;
                                        350, 600,    40;
                                        480, 450,    20;
                                        700, 700,    40;
                                        720, 760,    20;
                                        680, 760,    20;
                                        200, 400,    40;
                                        200, 520,    40;
                                        200, 600,    20;
                                        500, 200,    40;
                                        700, 200,    40;  ];   % 导弹、火炮、气象等威胁


% 无人机约束设置（min,max)
% （可单独为每个无人机设置，每行为一个无人机约束的上下限）
UAV.limt.v = 0.34*repmat([0.3, 0.7], UAV.num, 1);                 % 速度约束 （0.3Ma ~ 0.7Ma）
UAV.limt.phi = deg2rad(repmat([-60, 60], UAV.num, 1));      % 偏角约束 （-60° ~ 60°）
UAV.limt.theta = deg2rad(repmat([-45, 45], UAV.num, 1));   % 倾角约束 （-45° ~ 45°）
UAV.limt.h = repmat([0.02, 20], UAV.num, 1);                         % 高度约束 （0.02km ~ 20km）
UAV.limt.x = repmat([0, 875], UAV.num, 1);                            % 位置x约束 （0 ~ 875km）
UAV.limt.y = repmat([0, 875], UAV.num, 1);                            % 位置y约束 （0 ~ 875km）
UAV.limt.z = UAV.limt.h;                                                            % 位置z约束 （忽略地球弧度）
UAV.limt.L = zeros(UAV.num, 2);                                              % 航程约束 （最短航迹片段2km，最大航程1.5倍起始距离）
for i =1:UAV.num
    zz.max = 1.5 * norm(UAV.G(i, :) - UAV.S(i, :));
    zz.min = 2;
    UAV.limt.L(i, :) = [zz.min, zz.max];
end


% 多无人机协同设置
% （说明略）
UAV.tc = 6850;        % 协同时间 （单位s）
UAV.ds = 25;           % 安全距离 （单位km）


% 报错
ErrorCheck(UAV)
end





%% 程序自检
function ErrorCheck(UAV)

dim = UAV.PointDim; 
if dim ~= size(UAV.G,2) || dim ~= size(UAV.Menace.radar,2)-1 || dim ~= size(UAV.Menace.other,2)-1
    if dim ~= size(UAV.G,2)
        error('仿真维度为%d，但目标点坐标为%d维', dim, size(UAV.G,2))
    else
        error('仿真维度为%d，但威胁点坐标为%d维', dim, size(UAV.Menace.radar,2)-1)
    end
end

num = UAV.num;
if num ~= size(UAV.G,1) || num ~= size(UAV.limt.v,1) || num ~= size(UAV.limt.phi,1) ...
        || num ~= size(UAV.limt.theta,1) || num ~= size(UAV.limt.h,1) || num ~= size(UAV.limt.x,1) ...
        || num ~= size(UAV.limt.y,1) || num ~= size(UAV.limt.z,1) || num ~= size(UAV.limt.L,1)
    if num ~= size(UAV.G,1)
        error('无人机个数为%d, 但目标点有%d个', num, size(UAV.G,1))
    else
        error('约束条件个数与无人机个数不一致')
    end
end

if num ~= size(UAV.PointNum, 1)
    error('无人机个数为%d, 但为%d个无人机设置了导航点', num, size(UAV.PointNum, 1))
end

MaxPoint = floor(UAV.limt.L(:,2) ./ UAV.limt.L(:,1)) - 1;   % 每个无人机支持的最大航迹点数量
for i = 1 : UAV.num
    if UAV.PointNum(i) > MaxPoint(i)
        error('%d号无人机导航点个数超出任务需求，请尝试减少导航点个数', i)
    end
end

end
