function solution = MP_GWO(UAV, SearchAgents, Max_iter)
%MP_GWO 多种群灰狼优化算法
%Multi Population Gray Wolf Optimization

% 超参数
g = 50;       % 动态更新加权系数

% 算法初始化
[WolfPops, ~] = PopsInit(UAV, SearchAgents, false);   % 随机生成 初始狼群
ClassPops = PopsCluster(WolfPops, UAV);               % 进行初始聚类（用来获得 k 值）
dim = WolfPops.PosDim;                                % 状态变量维度
cSearchAgents = ClassPops.SearchAgents;               % 搜索智能体个数（子种群数量）
SearchAgents = cSearchAgents * ClassPops.k;           % 对所有智能体个数进行修正（k的整数倍）
WolfPops.Pos = WolfPops.Pos(1:SearchAgents, :);       % 对种群进行修正

% 报错
if cSearchAgents < 4
    error('搜索智能体个数过少')
end 

% 初始化解
Alpha_pos = zeros(ClassPops.k, dim);          % α解
Alpha_score = 1 ./ zeros(ClassPops.k, 1);     % α解适应度

Beta_pos = zeros(ClassPops.k, dim);            % β解
Beta_score = 1 ./ zeros(ClassPops.k, 1);       % β解适应度

Delta_pos = zeros(ClassPops.k, dim);           % δ解
Delta_score = 1 ./ zeros(ClassPops.k, 1);      % δ解适应度

Fitness_list = zeros(ClassPops.k, Max_iter);  % 适应度曲线

Pops.PosDim = WolfPops.PosDim;  % 子种群
Pops.lb = WolfPops.lb;
Pops.ub = WolfPops.ub;

% 迭代求解
tic
fprintf('>>MP-GWO 优化中    00.00%%')
for iter = 1 : Max_iter

    % ①  更新参数a
    a = 2 - iter * 2 / Max_iter;          % 线性递减
    %a = 2 * cos((iter/Max_iter)*pi/2);   % 非线性递减
    
    % ②  聚类
    if iter > 1 %初始化时聚完一次了，节省计算量
    ClassPops = PopsCluster(WolfPops, UAV);
    end
    for k = 1 : ClassPops.k
        Positions = ClassPops.Pos{k};  

        % ③  寻找 α、β、δ 狼
        for i = 1 : cSearchAgents
            % 读取目标函数
            fitness = ClassPops.Fitness(k, i);

            % 更新 Alpha、Beta 和 Delta 解
            if fitness <= Alpha_score(k)  % 适应能力最强（因为性能指标越小越好，因此为小于号）
                Alpha_score(k) = fitness;
                Alpha_pos(k, :) = Positions(i, :);
            end 
            if fitness > Alpha_score(k) && fitness <= Beta_score(k)
                Beta_score(k) = fitness;
                Beta_pos(k, :) = Positions(i, :);
            end
            if fitness > Alpha_score(k) && fitness > Beta_score(k) && fitness <= Delta_score(k)
                Delta_score(k) = fitness;
                Delta_pos(k, :) = Positions(i, :);
            end
        end

        % ④  更新位置（朝着前三只狼位置前进）
        for i = 1 : cSearchAgents
            for j = 1 : dim
    
                r1 = rand();
                r2 = rand();
                A1 = 2*a*r1 - a;
                C1 = 2*r2;
                D_alpha = abs(C1*Alpha_pos(k, j) - Positions(i, j));
                X1 = Alpha_pos(k, j) - A1*D_alpha;
    
                r1 = rand();
                r2 = rand();            
                A2 = 2*a*r1 - a;
                C2 = 2*r2;
                D_beta = abs(C2*Beta_pos(k, j) - Positions(i, j));
                X2 = Beta_pos(k, j) - A2*D_beta;
                
                r1 = rand();
                r2 = rand();
                A3 = 2*a*r1 - a;
                C3 = 2*r2;
                D_delta = abs(C3*Delta_pos(k, j) - Positions(i, j));
                X3 = Delta_pos(k, j) - A3*D_delta;
                
                % 静态更新
                Positions(i, j) = (X1 + X2 + X3) / 3;
                % 动态更新
                %                 q = g * a; %阈值
                %                 if abs(Alpha_score(k)-Delta_score(k)) > q
                %                     Sum_score = Alpha_score(k) + Beta_score(k) + Delta_score(k);
                %                     Positions(i, j) = (Alpha_score(k)*X1 + Beta_score(k)*X2 + Delta_score(k)*X3) / Sum_score;
                %                 else
                %                     Positions(i, j) = (X1 + X2 + X3) / 3;
                %                 end

            end
        end

    % ⑤  调整不符合要求的状态变量
    Pops.Pos = Positions;
    ProbPoints =  ClassPops.ProbPoints{k};
    [Pops, ~] = BoundAdjust(Pops, ProbPoints, UAV);

    % ⑥  存储适应度
    Fitness_list(k, iter) = Alpha_score(k);

    % ⑦  合并种群
    WolfPops.Pos(cSearchAgents*(k-1)+1:cSearchAgents*k, :) = Pops.Pos;

    end

    if iter/Max_iter*100 < 10
        fprintf('\b\b\b\b\b%.2f%%', iter/Max_iter*100)
    else
        fprintf('\b\b\b\b\b\b%.2f%%', iter/Max_iter*100)
    end
end
fprintf('\n\n>>计算完成！\n\n')
toc


% 寻找 α β δ 位置
n = 3; 
A = ClassPops.Fitness; 
t = findmin(A, n);
index = cSearchAgents * (t(:, 1) - 1) + t(:, 2);

real_Alpha_no = index(1);
real_Beta_no = index(2);
real_Delta_no = index(3);
Alpha_Data = ClassPops.Data{t(1, 1)}{t(1, 2)}  ;

% 输出值
solution.method = 'MP-GWO';                                % 算法
% solution.ClassPops = ClassPops;                          % 分类信息
solution.WolfPops = WolfPops;                              % 所有解种群信息
solution.Tracks = Pops2Tracks(WolfPops, UAV);              % 所有解航迹信息
solution.Fitness_list = mean(Fitness_list, 1);             % 所有α解的平均适应度曲线
solution.Alpha_Data = Alpha_Data;                          % 真 · α 的威胁信息
solution.Alpha_no = real_Alpha_no;                         % 真 · α 的位置
solution.Beta_no = real_Beta_no;                           % 真 · β 的位置
solution.Delta_no = real_Delta_no;                         % 真 · δ 的位置

end



%% 寻找A矩阵中最小n个数的位置
function t = findmin(A, n)
    t = sort(A(:));
    [x, y] = find(A <= t(n), n);
    t = [x, y];         % 前n个最小项在矩阵A中的位置[行,列]
    B = zeros(n, 1);
    for i = 1 : n
        B(i) = A(t(i, 1), t(i, 2));
    end
    [~, index] = sort(B);
    t = t(index, :); % 前n个从小到大排序的位置
end



%% 对种群进行聚类
function [ClassPops] = PopsCluster(WolfPops, UAV)

SearchAgents = size(WolfPops.Pos, 1);  % 智能体个数 
Dim = WolfPops.PosDim;                % 智能体维度
Tracks = Pops2Tracks(WolfPops, UAV); % 智能体转换成航迹信息

% 计算适应度
o_Fitness = zeros(SearchAgents, 1); % 60*1
o_subF = []; % 5*60
o_ProbPoints = cell(SearchAgents, 1);  % 60*1
o_Data = cell(SearchAgents, 1); % 60*1

% parfor 并行计算适应度
for i = 1:SearchAgents
    [fitness, subF, Data] = ObjFun(Tracks{i}, UAV);
    o_ProbPoints{i} = Data.ProbPoint;  %cell-cell
    o_Data(i) = {Data}; %cell-struct
    o_Fitness(i) = fitness; %vector-var
    o_subF = [o_subF, subF]; 
end

% 分类
k = size(subF, 1);                          % 分 k 类（由objfun决定）
cSearchAgents = floor(SearchAgents / k);    % 并行智能体个数
cFitness = zeros(k, cSearchAgents);         % 保存每类的适应度
cPositions = cell(k, 1);                    % 保存每类的位置信息
cTracks = cell(k, 1);                       % 保存每类的航迹信息
cProbPoints = cell(k, 1);                   % 保存每类的有问题航迹点
cData = cell(k, 1);                         % 存储每类的检测报告

% 排序
[~, Index] = sort(o_subF, 2, "ascend") ;     % 沿维度2升序排序，返回新矩阵和序号
                                             % fitness越小越好
% 聚类
for i = 1:k
    Positions = zeros(cSearchAgents, Dim);
    batchTrack = cell(cSearchAgents, 1);
    batchProbPoints = cell(cSearchAgents, 1);
    batchData = cell(cSearchAgents, 1);
    for j = 1:cSearchAgents
        idx = Index(i, j);
        cFitness(i, j) = o_Fitness(idx); %mat-vector
        Positions(j, :) = WolfPops.Pos(idx, :); %mat-mat
        batchTrack{j} = Tracks{idx}; %cell-cell
        batchProbPoints{j} = o_ProbPoints{idx}; %cell-cell
        batchData{j} = o_Data{idx}; %cell-cell
    end
    cPositions(i) = {Positions}; %cell-mat
    cTracks{i} = batchTrack; %cell-cell
    cProbPoints{i} = batchProbPoints; %cell-cell
    cData{i} = batchData; %cell-cell
end

% 输出
ClassPops.Pos = cPositions;
ClassPops.Tracks = cTracks;
ClassPops.ProbPoints = cProbPoints;
ClassPops.Data = cData;
ClassPops.Fitness = cFitness;
ClassPops.SearchAgents = cSearchAgents;
ClassPops.k = k;
end

