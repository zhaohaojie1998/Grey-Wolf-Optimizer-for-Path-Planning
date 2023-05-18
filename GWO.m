function solution = GWO(UAV, SearchAgents, Max_iter)
%GWO 灰狼优化算法
%Gray Wolf Optimization

% 超参数
g = 50;       % 动态加权系数

% 算法初始化
[WolfPops, Tracks] = PopsInit(UAV, SearchAgents, false);   % 随机生成 初始狼群 和 轨迹们
dim = WolfPops.PosDim;                                     % 状态变量维度

% 初始化解
Alpha_pos = zeros(1, dim);     % α解
Alpha_score = inf;             % α解适应度
Alpha_no = 1;                  % α解编号

Beta_pos = zeros(1, dim);      % β解
Beta_score = inf;              % β解适应度
Beta_no = 1;                   % β解编号

Delta_pos = zeros(1, dim);     % δ解
Delta_score = inf;             % δ解适应度
Delta_no = 1;                  % δ解编号

Fitness_list = zeros(1, Max_iter);

% 迭代求解
tic
fprintf('>>GWO 优化中    00.00%%')
for iter = 1 : Max_iter

    % ①  计算每只狼的适应度，更改其种群等级
    ProbPoints = cell(SearchAgents, 1);   
    for i = 1 : SearchAgents
        % 计算目标函数
        [fitness, ~, Data] = ObjFun(Tracks{i}, UAV);    % 一个智能体的目标函数
        ProbPoints{i} = Data.ProbPoint;                 % 所有智能体不符合条件的状态

        % 更新 Alpha、Beta 和 Delta 解
        if fitness <= Alpha_score  % 适应能力最强（因为性能指标越小越好，因此为小于号）
            Alpha_score = fitness;
            Alpha_pos = WolfPops.Pos(i, :);
            Alpha_no = i;
        end 
        if fitness > Alpha_score && fitness <= Beta_score
            Beta_score = fitness;
            Beta_pos = WolfPops.Pos(i, :);
            Beta_no = i;
        end
        if fitness > Alpha_score && fitness > Beta_score && fitness <= Delta_score
            Delta_score = fitness;
            Delta_pos = WolfPops.Pos(i, :);
            Delta_no = i;
        end
    end

    % ②  更新参数a
    a = 2 - iter * 2 / Max_iter;              % 线性递减
    %a = 2 * cos((iter / Max_iter) * pi/2);   % 非线性递减

    % ③  更新位置（朝着前三只狼位置前进）
%     r1 = rand(SearchAgents, dim);
%     r2 = rand(SearchAgents, dim);
%     A1 = 2*a*r1 - a;
%     C1 = 2*r2;
%     D_alpha = abs(C1.*repmat(Alpha_pos, SearchAgents, 1) - WolfPops.Pos);
%     X1 = repmat(Alpha_pos, SearchAgents, 1) - A1.*D_alpha;
% 
%     r1 = rand(SearchAgents, dim);
%     r2 = rand(SearchAgents, dim);            
%     A2 = 2*a*r1 - a;
%     C2 = 2*r2;
%     D_beta = abs(C2.*repmat(Beta_pos, SearchAgents, 1) - WolfPops.Pos);
%     X2 = repmat(Beta_pos, SearchAgents, 1) - A2.*D_beta;
%     
%     r1 = rand(SearchAgents, dim);
%     r2 = rand(SearchAgents, dim);
%     A3 = 2*a*r1 - a;
%     C3 = 2*r2;
%     D_delta = abs(C3.*repmat(Delta_pos, SearchAgents, 1) - WolfPops.Pos);
%     X3 = repmat(Delta_pos, SearchAgents, 1) - A3.*D_delta;
% 
%     %-- 静态更新位置
%     WolfPops.Pos = (X1 + X2 + X3) / 3;  
    %-- 动态更新位置
%     q = g * a; 
%     if abs(Alpha_score - Delta_score) > q
%         Sum_score = Alpha_score + Beta_score + Delta_score;
%         WolfPops.Pos = (Alpha_score*X1 + Beta_score*X2 + Delta_score*X3) / Sum_score;
%     else
%         WolfPops.Pos = (X1 + X2 + X3) / 3;
%     end

    for i = 1 : SearchAgents
        for j = 1 : dim

            r1 = rand();
            r2 = rand();
            A1 = 2*a*r1 - a;
            C1 = 2*r2;
            D_alpha = abs(C1*Alpha_pos(j) - WolfPops.Pos(i, j));
            X1 = Alpha_pos(j) - A1*D_alpha;

            r1 = rand();
            r2 = rand();            
            A2 = 2*a*r1 - a;
            C2 = 2*r2;
            D_beta = abs(C2*Beta_pos(j) - WolfPops.Pos(i, j));
            X2 = Beta_pos(j) - A2*D_beta;
            
            r1 = rand();
            r2 = rand();
            A3 = 2*a*r1 - a;
            C3 = 2*r2;
            D_delta = abs(C3*Delta_pos(j) - WolfPops.Pos(i, j));
            X3 = Delta_pos(j) - A3*D_delta;
            
            %静态更新
            WolfPops.Pos(i, j) = (X1 + X2 + X3) / 3;
%             %动态更新
%             %              q = g * a; %阈值
%             %              if abs(Alpha_score - Delta_score) > q
%             %                     Sum_score = Alpha_score + Beta_score + Delta_score;
%             %                     WolfPops.Pos(i, j) = (Alpha_score*X1 + Beta_score*X2 + Delta_score*X3) / Sum_score;
%             %              else
%             %                     WolfPops.Pos(i, j) = (X1 + X2 + X3) / 3;
%             %              end
        end
    end

    % ④  调整不符合要求的状态变量
    [WolfPops, Tracks] = BoundAdjust(WolfPops, ProbPoints, UAV);

    % ⑤  存储适应度
    Fitness_list(iter) = Alpha_score;


    if iter/Max_iter*100 < 10
        fprintf('\b\b\b\b\b%.2f%%', iter/Max_iter*100)
    else
        fprintf('\b\b\b\b\b\b%.2f%%', iter/Max_iter*100)
    end
end
fprintf('\n\n>>计算完成！\n\n')
toc




% 输出值
solution.method = 'GWO';                % 算法
solution.WolfPops = WolfPops;           % 所有解种群信息
solution.Tracks = Tracks;               % 所有解航迹信息
solution.Fitness_list = Fitness_list;   % α解适应度曲线
solution.Alpha_Data = Data;             % α解的威胁信息
solution.Alpha_no = Alpha_no;           % α解的位置
solution.Beta_no = Beta_no;             % β解的位置
solution.Delta_no = Delta_no;           % δ解的位置

end

