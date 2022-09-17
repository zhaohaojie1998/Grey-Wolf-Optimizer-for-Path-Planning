function solution = CS_GWO(UAV, SearchAgents, Max_iter)
%CS_GWO 灰狼-布谷鸟优化算法
%Gray Wolf Cuckoo Optimization

% 超参数
pa = 0.25;   % 布谷鸟搜索参数

% 算法初始化
[WolfPops, Tracks] = PopsInit(UAV, SearchAgents, false);   % 随机生成 初始狼群 和 轨迹们
dim = WolfPops.PosDim;                                                         % 状态变量维度

% 初始化解
Alpha_pos = zeros(1, dim);   % α解
Alpha_score = inf;                  % α解适应度
Alpha_no = 1;                         % α解编号

Beta_pos = zeros(1, dim);      % β解
Beta_score = inf;                     % β解适应度
Beta_no = 1;                             % β解编号

Delta_pos = zeros(1, dim);     % δ解
Delta_score = inf;                    % δ解适应度
Delta_no = 1;                           % δ解编号

Fitness_list = zeros(1, Max_iter);

% 迭代求解
tic
fprintf('>>CS_GWO 优化中    00.00%%')
for iter = 1 : Max_iter

    % ①  计算每只狼的适应度，更改其种群等级
    ProbPoints = cell(SearchAgents, 1);   
    for i = 1 : SearchAgents
        % 计算目标函数
        [fitness, ~, Data] = ObjFun(Tracks{i}, UAV);    % 一个智能体的目标函数
        ProbPoints{i} = Data.ProbPoint;                       % 所有智能体不符合条件的状态

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
    a = 2 - iter * 2 / Max_iter;                  % 线性递减
    %a = 2 * cos((iter / Max_iter) * pi/2);   % 非线性递减

    % ③  更新位置（朝着前三只狼位置前进）
    for i = 1 : SearchAgents
        for j = 1 : dim

            r1 = rand();
            r2 = rand();
            A1 = 2*a*r1 - a;
            C1 = 2*r2;
            D_alpha = abs(C1*Alpha_pos(j) - WolfPops.Pos(i, j));
            X1(i, j) = Alpha_pos(j) - A1*D_alpha;

            r1 = rand();
            r2 = rand();            
            A2 = 2*a*r1 - a;
            C2 = 2*r2;
            D_beta = abs(C2*Beta_pos(j) - WolfPops.Pos(i, j));
            X2(i, j) = Beta_pos(j) - A2*D_beta;
            
            r1 = rand();
            r2 = rand();
            A3 = 2*a*r1 - a;
            C3 = 2*r2;
            D_delta = abs(C3*Delta_pos(j) - WolfPops.Pos(i, j));
            X3(i, j) = Delta_pos(j) - A3*D_delta;
            
            %更新
            %WolfPops.Pos(i, j) = (X1(i, j) + X2(i, j) + X3(i, j)) / 3;
 
        end

    end

    % ④  Cuckoo 搜索
    fitness = nan(SearchAgents, 1);
    for i = 1 : SearchAgents
        [fitness(i), ~, ~] = ObjFun(Tracks{i}, UAV);
    end
    [~, index] = min(fitness);
    best = WolfPops.Pos(index, :);
    X1 = get_cuckoos(X1, best, WolfPops.lb, WolfPops.ub); 
    X2 = get_cuckoos(X2, best, WolfPops.lb, WolfPops.ub);
    X3 = get_cuckoos(X3, best, WolfPops.lb, WolfPops.ub);
    X1 = empty_nests(X1, WolfPops.lb, WolfPops.ub, pa);
    X2 = empty_nests(X2, WolfPops.lb, WolfPops.ub, pa);
    X3 = empty_nests(X3, WolfPops.lb, WolfPops.ub, pa);
    WolfPops.Pos = (X1 + X2 + X3) / 3;

    % ⑤  调整不符合要求的状态变量
    [WolfPops, Tracks] = BoundAdjust(WolfPops, ProbPoints, UAV);

    % ⑥  存储适应度
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
solution.WolfPops = WolfPops;       % 所有解种群信息
solution.Tracks = Tracks;                  % 所有解航迹信息
solution.Fitness_list = Fitness_list;   % α解适应度曲线
solution.Alpha_Data = Data;            % α解的威胁信息
solution.Alpha_no = Alpha_no;        % α解的位置
solution.Beta_no = Beta_no;             % β解的位置
solution.Delta_no = Delta_no;          % δ解的位置

end



%%%%% 布谷鸟搜索 %%%%%
function nest = get_cuckoos(nest, best, Lb, Ub)
    n = size(nest, 1);
    beta = 3/2;
    sigma = (gamma(1 + beta) * sin(pi*beta/2)/(gamma((1 + beta)/2)*beta*2^((beta - 1)/2)))^(1/beta);
    for j = 1:n
        s = nest(j, :);
        u = randn(size(s))*sigma;
        v = randn(size(s));
        step = u./abs(v).^(1/beta);
        stepsize = 0.01*step.*(s - best);
        s = s + stepsize.*randn(size(s));
        nest(j, :) = BoundClamp(s, Lb, Ub);
    end
end
function new_nest = empty_nests(nest, Lb, Ub, pa)
    n = size(nest, 1);
    K = rand(size(nest)) > pa;
    stepsize = rand*(nest(randperm(n), :) - nest(randperm(n), :));
    new_nest = nest + stepsize.*K;
    for j = 1:size(new_nest, 1)
        s = new_nest(j, :);
        new_nest(j, :) = BoundClamp(s, Lb, Ub);
    end
end
function x = BoundClamp(x, lb, ub)
    Flag4ub = x > ub;
    Flag4lb = x < lb;
    x = x .* ( ~(Flag4ub + Flag4lb) ) + ub .* Flag4ub + lb .* Flag4lb;
end
