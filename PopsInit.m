function [WolfPops, Tracks] = PopsInit(UAV, SearchAgents, GreedyInit)
%POPSINIT 初始化狼群"位置"
if (nargin<3)
GreedyInit = false;
end

% 〇 程序自检
if SearchAgents < 4
    error('搜索智能体个数过少')
end


% ① 随机初始化解空间
dim = UAV.PointDim;  % 坐标轴维度 

% 速度 n个无人机
lb_v = UAV.limt.v(:, 1)' ;    % 1*n
ub_v = UAV.limt.v(:, 2)' ;   % 1*n
V0 = rand(SearchAgents, UAV.num) .* repmat(ub_v - lb_v, SearchAgents, 1) + repmat(lb_v, SearchAgents, 1);
%  对于UAV，每一行表示每架飞机的参数
%  对于Agent，每一行表示每个智能体的参数，列表示飞机的参数平铺

% 航迹点 
P0 = [];
for i = 1 : UAV.num
    lb = [UAV.limt.x(i, 1), UAV.limt.y(i, 1), UAV.limt.z(i, 1)] ;     % 1*3 维
    ub = [UAV.limt.x(i, 2), UAV.limt.y(i, 2), UAV.limt.z(i, 2)] ;    % 1*3 维
    if dim < 3
        lb = lb(1:2);      % 1*2 维
        ub = ub(1:2);    % 1*2 维
    end
    PointNum = UAV.PointNum(i);                  % 航迹点个数
    P_i = rand(SearchAgents, dim*PointNum) .* repmat(ub - lb, SearchAgents, PointNum) + repmat(lb, SearchAgents, PointNum);
   
    % 贪婪生成法（从第二个航迹点开始生成贪婪点）
    if GreedyInit  
        for k = 2 : PointNum    
            ep = 0.2;      % 阈值
            m = rand();  % 变异量
            if m >= ep   % 当变异参数超过阈值时，第k个航迹点贪婪生成，否则随机生成
            % 下限是上一个航迹点，上限是终点
                lb = P_i(:, dim*(k-1)-dim+1:dim*(k-1));         % N*2维
                ub = repmat(UAV.G(i, :), SearchAgents, 1);   % N*2维
                P_i(:, dim*k-dim+1:dim*k) = rand(SearchAgents, dim) .* (ub-lb) + lb;
            end
        end
    end

    P0 = [P0, P_i];
end

WolfPops.Pos = [P0, V0]; % 狼群位置


% ② 狼群位置上下限
lb = [];
ub = [];
for i = 1 : UAV.num
    lb_i = [UAV.limt.x(i, 1), UAV.limt.y(i, 1), UAV.limt.z(i, 1)] ;     % 1*3 维
    ub_i = [UAV.limt.x(i, 2), UAV.limt.y(i, 2), UAV.limt.z(i, 2)] ;    % 1*3 维
    if dim < 3
        lb_i = lb_i(1:2);      % 1*2 维
        ub_i = ub_i(1:2);    % 1*2 维
    end
    PointNum = UAV.PointNum(i);
    lb_i = repmat(lb_i, 1, PointNum);
    ub_i = repmat(ub_i, 1, PointNum);
    
    lb = [lb, lb_i];
    ub = [ub, ub_i];
end
lb = [lb, lb_v];
ub = [ub, ub_v];

WolfPops.PosDim = size(lb, 2);
WolfPops.lb = lb;
WolfPops.ub = ub;


% ③ 生成初始航迹
Tracks = Pops2Tracks(WolfPops, UAV); 

clc
fprintf('>>算法初始化完成！\n\n')

end
