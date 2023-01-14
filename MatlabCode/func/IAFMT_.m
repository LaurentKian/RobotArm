function [z_r,VA,Con,t,cost,additional_out]=IAFMT_(xi,Xg,s)
%% IAFMT规划器//采用均匀采样
%--------------------------------------------------------------------------
% 输入:  
%       xi: 1*2 double      起点
%       Xg: n*2 double      终点区域
%       s:  struct          仿真设计


% 输出:    
%       z:      double            路径终点序号
%       V:      n*n double        树的点集
%       Con:    n*n struct      树的边集

%--------------------------------------------------------------------------
% 参数设置
eta = 0.2;     %用于计算半径的参数
rn = 0;        %初始半径
n0=50;         %初始采样点数量
n_re=50;       %补充采样点数目
J_g=0;          %成本阈值


Cap=4e4;        %节点的最大数目
%--------------------------------------------------------------------------

%% 初始化
NODE = 1;                   % 点序号所在列
POS = 2:7;                  % 点坐标所在列
COST = 8;                   % 路径成本所在列
PAR = 9;                    % 父节点所在列
EXPAND = 10;                % 拓展次数所在列
HEUR = 11;                  % 启发函数值所在列
START = 1;                  % 起点所在行
GOAL = 2;                   % 终点所在行
COL = 10;                   % 数据矩阵列数
CHECK_COL = 10;             % 依据启发函数值判断
N_MAX = 500;                % 临近点预分配空间


is_heuristic = true;       %是否采用启发式算法
col = 11;                    %节点矩阵列数
check_col = 11;              %依据树成本判断
    
% 关节空间的范围
joint_range = s.joint_range;

% 点基本信息矩阵
% |  1   |  2   -   7  |    8   |   9    |    10    | ... | 11 |
% | Node |    joints   | cost_T | Parent |  Expandi | ... | f  |
VA=NaN*ones(Cap,col);
VA(:,NODE)=1:Cap;  % 序号顺序赋值
VA(:,EXPAND)=0;      % 所有点均未经过拓展
VAi=0;


a_VA=false(Cap,1); % 裁剪后有效点逻辑索引
V_un=VA(a_VA);   % 无效点序号

% 点的连接关系矩阵
% |  1   |  2   |   3    |  4   | 5  |
% | Node | Near | cost_e | cost | Ni |
near_n = 800; % 附近点数目
Con=repmat(struct("Node",NaN,"Near",NaN*ones(near_n,1),"cost_e",...
    NaN*ones(near_n,1),"cost",NaN*ones(near_n,1),'Ni',0),[Cap,1]);


% 三个点集
Vu=Array();
Vo=Array();
Vc=Array();

% 起点作为1号点
Vo.add(START);

if is_heuristic
    % 用作求解启发函数值的Xg的中心点xg
    xg=Xg;
    VA(START,:)=[1,xi',0,0,0,norm(xi-xg)];
else
    xg=Xg;
    VA(START,:)=[1,xi',0,0,0];
end

VAi=VAi+1;


% 终点区域随机点为2号点
% Vu.add(2);
% VA(2,2:3)=Xg(randperm(size(Xg,1),1),:);
% VAi=VAi+1;
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vu.add(GOAL);
VA(GOAL,POS)=xg';
VAi=VAi+1;

Xg=Xg(ceil(size(Xg,1)/2),:);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% 初始解与优化解flag
sig=0;          

% 初始化Von
Von=Array();

% 迭代次数(批量采样次数)
it=0;

% 各次迭代末端采样点
Sample_end=zeros(50,1);

z=1;

% 目标区域的成本最低点及其成本
z_Xg_min=NaN;
c_best = Inf;

% xi - xg 线段与世界坐标所成角度
vector = xg - xi;

% 起点至终点的直线距离,初始化最佳距离
c_min = norm(xi - xg);

% 有效采样点总数
n_a = 0;

%% 主程序
while it<10 
    if sig==0
        Sample(n0);
    
        it=it+1;
        Sample_end(it+1)=VAi;
        a_VA(1:VAi)=1;
        
        
        HybridSearch();
        Eps = 0.01; %容差
        if VA(z_Xg_min,COST)<=J_g+Eps
            break;
        end
        
    else 
        DynamicOS();
        
        % 判断终点成本是否小于设定阈值
        Eps = 0.01; %容差
        if VA(z_Xg_min,COST)<=J_g+Eps 
            break;
        end
    end
end

z_r=z_Xg_min;
t=toc;
cost=VA(z_r,COST);
if is_heuristic
    fprintf("IAFMT_H:最终解成本为 %0.2f,用时 %0.2f s\n",cost,t)
else
    fprintf("IAFMT:最终解成本为 %0.2f,用时 %0.2f s\n",cost,t)
end

additional_out={VAi,VA};




%% 搜寻初始解
    function HybridSearch()

        while isempty(intersect(VA(z,POS),xg','rows'))

            z=ExpandTree(z);
            
            %初始采样点无解则补充采样点
            if Vo.num==0 && sig==0  
                InsertNode();
            end
        end
        
        Vc.add(z);
        z_Xg_min=z;

        c_best = VA(z_Xg_min,COST);
        sig=1;
        
        
if is_heuristic
        fprintf("\nIAFMT_H:完成初始解搜索,初始解成本为%0.2f\n",...
            VA(z_Xg_min,COST))
else
        fprintf("\nIAFMT:完成初始解搜索,初始解成本为%0.2f\n",...
            VA(z_Xg_min,COST))
end 

end
%% 路径优化
    function DynamicOS()

            Prune(VA(z_Xg_min,COST));
            
            Sample(n_re);
           
            it=it+1;
            Sample_end(it+1)=Sample_end(it)+n_re;
            a_VA(Sample_end(it)+1:VAi)=1;

            
            % 更新 Vo Vc Vu

            Vo.add(Vc);
            Vo.rm(V_un);
            Vu.rm(V_un);
            Vc.clear();
            z=1;

            if is_heuristic
            % 如果启用启发式算法，则当树成本高于当前最优解时终止迭代
                while Vo.num~=0 && VA(z,HEUR)<=c_best

                    z=ExpandTree(z);

                    if isempty(z)
                        disp("z为空")
                    end
                    
                end    
            else
                while Vo.num~=0
                    z=ExpandTree(z);
                end
            end
            
            vc=Vc.show();
            [~,z_Xg_i]=intersect(VA(vc,POS),Xg,'rows');
            z_Xg=vc(z_Xg_i);
            [~,zi]=min(VA(z_Xg,COST));
            z_Xg_min=z_Xg(zi);
            c_best=VA(z_Xg_min,COST);
            
if is_heuristic            
            fprintf("IAFMT_H:本次迭代最佳成本为%0.2f\n",VA(z_Xg_min,COST))
else
            fprintf("IAFMT:本次迭代最佳成本为%0.2f\n",VA(z_Xg_min,COST))
end

end

%% 树拓展
    function z_n=ExpandTree(z)

        % 重置Von
        Von.clear();
        
        N_z=Near(z,rn);

        if isempty(N_z)
            X_near=[];
        else
            X_near=N_z(sum(N_z==(Vu.show())',2)>=1);
        end
        
        if ~isempty(X_near)
            for x=X_near'               
                
                % 找出Vo中到x成本最低的点
                N_x=Near(x,rn);

                ix=find(sum(N_x==(Vo.show())',2)>=1);
                Y_near=N_x(ix);

                [~,iy]=min(VA(Y_near,COST)+Con(x).cost_e(ix));
                ymin=Y_near(iy);
        
                % y 在 x 的Near队列中的索引
                x_y_i=Con(x).Near(1:Con(x).Ni)==ymin;
                
                % x 在 y 的Near队列中的索引
                y_x_i=Con(ymin).Near(1:Con(x).Ni)==x;
                
                if CollF(ymin,x)
                    % 更新子节点的实际成本
                    Con(x).cost(x_y_i)=Con(x).cost_e(x_y_i);
                    % 添加子节点的父节点
                    VA(x,PAR)=ymin;
                    % 添加子节点成本
                    VA(x,COST)=VA(ymin,COST)+Con(x).cost(x_y_i);
                    if is_heuristic                       
                        % 更新子节点的启发函数值
                        VA(x,HEUR)=VA(x,COST)+norm(VA(x,POS)-xg);
                    end
                    
                        
                    % 更新父节点的实际成本
                    Con(ymin).cost(y_x_i)=Con(x).cost_e(y_x_i);
                    
                    % 将x从Vu移入入Von
                    Vu.rm(x);
                    Von.add(x);
                    
                    % 如果已经找到初始解，则进行路线重绘
                    if sig==1
                        Rewire(Y_near,x);
                    end
                    
                else
                    Con(x).cost(x_y_i)=Inf;
                    Con(ymin).cost(y_x_i)=Inf;
                end
            end      
        end
        % 更新Vo和Vc
        Vo.rm(z);
        Vc.add(z);
        Vo.add(Von);

        % 更新z
        [~,z_ni]=min(VA(Vo.show(),check_col));
        Vo_a=Vo.show();
        z_n=Vo_a(z_ni);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%检查三个点集是否正确
if Vo.num+Vc.num+Vu.num ~= sum(a_VA)
    disp("点集错误")
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
end


%% 单个采样点补充
    function InsertNode()
        while 1

            % 补充随机采样点
            x_s = zeros(1,6);
            for i = 1:6
                x_s(i) = random("Uniform",joint_range{i}(1),joint_range{i}(2));
            end
            vc=Vc.show();
            
            % 随机点到 closed 点集的距离
            Dis=dist(VA(vc,POS)-x_s);
            [dis,vci]=min(Dis);
            
            % 找到距离随机点最近的 closed 点
            vc_c=vc(vci);
            
            if dis>=rn  %不在rn范围内则进行steer
                x_n=VA(vc_c,POS)+(x_s-VA(vc_c,POS)*rn/dis);
                dis=norm(VA(vc_c,POS)-x_n);
            else
                x_n=x_s;
            end
            
            if ~check(x_n)
                continue
            end
            
            
            % 碰撞检测
            if CollF(vc_c,x_n)

                % 加入点集列表
                VA(VAi+1,POS)=x_n;
                VA(VAi+1,COST)=VA(vc_c,COST)+dis;
                a_VA(VAi+1)=1;
                VAi=VAi+1;
                
                % 为新采样点添加父节点
                VA(VAi,PAR)=vc_c;
                
                % 将新采样点加入Vo点集
                Vo.add(VAi);
                z=VAi;
                
                % 为附近已拓展点更新状态
                ve=VA(VA(1:VAi,EXPAND)==1,1); 
                dis_c=dist(VA(ve,POS)-VA(VAi,POS));
                
                if any(dis_c==0) % 判断采样点是否于Vc重合
                    continue
                end
                
                dis_c_r=dis_c(dis_c<=rn);
                x_n_near=ve(dis_c<=rn);
                
                for i=1:size(x_n_near,1)
                    ve_near=x_n_near(i);
                    Con(ve_near).Near(Con(ve_near).Ni+1)=VAi;
                    Con(ve_near).cost_e(Con(ve_near).Ni+1)=dis_c_r(i);
                    Con(ve_near).Ni=Con(ve_near).Ni+1;
                end
                
                Sample_end(2)=Sample_end(2)+1;

                n_a = n_a + 1;
                break
            end
               
        end
    end

%% 树重绘
    function Rewire(Y_near,x)

        
        iy=VA(Y_near,PAR)~=VA(x,PAR);   % 除去父节点相同的点
        H_near=Y_near(iy);
        
        for h=H_near'
            cost_re=VA(x,COST)+dist(VA(h,POS)-VA(x,POS));
            if cost_re<VA(h,COST)
                if CollF(x,h)

                    VA(h,PAR)=x;              % 将父节点改为x                 
                    sub=VA(h,COST)-cost_re;    
                       
                    % 更新父节点的成本
                    VA(h,PAR)=cost_re;
                    if is_heuristic
                        % 更新父节点启发函数值
                        VA(h,HEUR)=VA(h,COST)+norm(VA(h,POS)-xg);
                    end
                    
                    UpdateChildCosts(h,sub);
                end
            end
        end
    end

%% 碰撞检测
    function ret = CollF(x,v)
        x_pos = VA(x,POS);

        if size(v,2)==1         % 判断输入参数是点的序号还是点的坐标
            v_pos=VA(v,POS);
        else
            v_pos=v;
        end
        
        ret = true;
        % 求出单位向量
        vec = (v_pos - x_pos) / norm(v_pos - x_pos);
        % 求出增量
        dl = deg2rad(2);
        sdl = 0;
        dis = norm(v_pos - x_pos);
        while ~all(x_pos == v_pos)
            if sdl < dis
                sdl = sdl + dl;
                x_pos = x_pos + vec*dl;
            else
                x_pos = v_pos;
            end
            setconfig(s,x_pos);
            if checkcollision(s)
                ret = false;
                break 
            end
        end
    end

%% 两点间距离（用于数组）
    function D=dist(V)
        D=sqrt(sum(V.^2,2));
    end
%% 寻找临近点子函数
function N_z_r=Near(z,rn)

    %判断该点在第几次迭代中拓展，仅遍历之后加入的点
    Expandi=VA(z,EXPAND);
    if Expandi==it   % 已经在该此迭代中拓展

       N_z_r=Con(z).Near(1:Con(z).Ni);

    else   
        if VA(z,EXPAND) == 0 %判断该点是否已进行过运算       
           Con(z).Node=z;
        end
        
        if sig==1   %如果已经经过知情集收缩，则去除附近点中的无效点
            
            i_re1=find(sum(Con(z).Near(1:Con(z).Ni)==V_un',2));
           
           
           % rn 随迭代变化,需删除附近点中距离超出当前rn的点
           i_re=Con(z).cost_e(1:Con(z).Ni)>rn;
           if any(i_re)
            i_re(i_re1)=1;
            i_re=find(i_re);
           else
            i_re=i_re1; 
           end
           
           Con(z).Near(i_re)=[];
           Con(z).cost_e(i_re)=[];
           Con(z).cost(i_re)=[];
           Con(z).Ni=Con(z).Ni-size(i_re,1);

        end
        vi=Sample_end(Expandi+1);       %已遍历点的末端
        VA_a_i=VA(vi+1:VAi,1);          %未遍历点
        VA_a_i=VA_a_i(a_VA(vi+1:VAi));  %未遍历点中有效点
        VA_a=VA(VA_a_i,:);              
        
        
        % 从未遍历点中筛选出有效点
        near_dis = dist(VA_a(:,POS)-VA(z,POS));
        N_z_i = near_dis<=rn;
        N_z_i(z) = 0;
        N_z = VA_a(N_z_i,NODE);
        N_size = size(N_z,1);
   
        % 更新z点的附近节点关系
        Con(z).Near(Con(z).Ni+1:Con(z).Ni+N_size)=N_z;
        Con(z).cost_e(Con(z).Ni+1:Con(z).Ni+N_size)=...
            near_dis(N_z_i);
        Con(z).Ni=Con(z).Ni+N_size;
        
        % 更新z点拓展时的迭代次数
        VA(z,EXPAND)=it;
        
        % 所有的附近节点
        N_z_r=Con(z).Near(1:Con(z).Ni);
        
    end                   
end

%% 裁剪树 
    function Prune(cost_g)
        
        % 无效点索引
        ip=dist(VA(1:VAi,POS)-xi')+dist(VA(1:VAi,POS)-VA(z_Xg_min,POS))...
            >cost_g;
        V_un=VA(ip,1);          % 更新无效点序号
        a_VA(1:VAi) = ~ip;        % 有效点索引
        VA(V_un,POS) = Inf;   % 更新无效点位置，成本
        VA(V_un,COST) = Inf;
        VA(V_un,PAR)=NaN;     % 删除无效点父节点

        
        % 去除无效点为前继点的信息
        v_un = V_un;
        VA_s = VA(1:VAi,PAR);
        ip_r = v_un;
        while true
            VA(ip_r,PAR)=NaN;
            
            Vo.rm(VA(ip_r,NODE));   % 父节点为无效点则移出Vo和Vc，移入Vu
            Vc.rm(VA(ip_r,NODE));
            Vu.add(VA(ip_r,NODE));
            
            ip_r = VA(sum(VA_s==v_un',2)>=1,1);
            v_un = VA(ip_r,NODE);
            if isempty(ip_r)
                break
            end

            VA(ip_r,PAR)=NaN;
            Vo.rm(VA(ip_r,NODE));   % 父节点为无效点则移出Vo和Vc，移入Vu
            Vc.rm(VA(ip_r,NODE));
            Vu.add(VA(ip_r,NODE));
        end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 %检查父节点有效性

 V_p=VA(2:VAi,PAR);
 V_p=rmmissing(V_p);
 
 % 所有父节点存在的节点
 VA_p = VA(V_p,:);

 % 找出父节点的前继点仍为空的点
 e_i = find(isnan(VA_p(:,PAR)),1);
 if ~isempty(e_i) 
     fprintf("存在失效父节点:%d\n",VA_p(e_i,1))
 end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        
n_a = sum(~ip);      % 更新总有效点数目
        
end
%% 知情集采样-
    function sample_inf = InfromedSampleFree(cost_g)
        % 计算旋转矩阵C和变形矩阵 L
                x_center = (xi+xg)/2;
                mat = eye(6);
                mat(:,1) = vector;
                C = schmidt_orthogonalization(mat);
                L = eye(6)*sqrt(cost_g^2-c_min^2)/2;
                L(1,1) = cost_g/2;
                
                % 单位球内均匀采样，需要化为列向量运算
                sample_inf = C*L*Sample_ball() + x_center; 
                sample_inf = sample_inf';
        
        % 判断采样点是否有效
        if ~check(sample_inf)
            sample_inf = []; 
        end
    end
%% 更新子节点成本
    function UpdateChildCosts(h,sub)
        vo=Vo.show();
        c_size=500;
        Children=zeros(c_size,1);   %添加子节点的队列
        Ci1=0;                      %子节点队列左指针
        
        vo_add=vo(VA(vo,PAR)==h);      % 寻找父节点为h的点
        vo(VA(vo,PAR)==h)=[];          % 去除已检索节点
        
        if ~isempty(vo_add)
            VA(vo_add,COST)=VA(vo_add,COST)-sub;  % 更新节点成本
            
            Children(Ci1+1:Ci1+size(vo_add,1))=vo_add;
            Ci2=Ci1+size(vo_add,1);         %子节点队列右指针
            
            while 1
                % 搜寻子节点
                ip=sum(VA(vo,PAR)==(Children(Ci1+1:Ci2))',2)>=1;  
                if any(ip)
                    vo_add=vo(ip);
                    vo(ip)=[]; % 若已经遍历其子节点，从队列中去除
                    VA(vo_add,COST)=VA(vo_add,COST)-sub;  % 更新节点成本
                    
                    % 更新节点启发函数值
                    if is_heuristic
                        VA(vo_add,HEUR)=VA(vo_add,COST)+dist(VA(vo_add,POS)-xg);
                    end
                    Children(Ci2+1:Ci2+size(vo_add,1))=vo_add;
                    
                    Ci1=Ci2;               % 更新子节点队列首尾指针
                    Ci2=Ci2+size(vo_add,1);
                else
                    break;
                end
            end
        end
    end
%% 更新搜索半径
    function Update_rn()
        
        % 全体采样空间的测度
        X_A = s.cspace_lebes;
        % 知情集的测度
        a = c_best/2; % 长轴
        b = sqrt(c_best^2 - norm(xi-xg)^2)/2; %  短轴
        X_inf = (2^6*a*b^5*gamma(1/6)^6)/(6^6*gamma(2));
        % 用作计算半径的体积
        X = min(X_A,X_inf);
        
        rn = 5*(1+eta)*2*(1/2)^(1/2)*(X/(pi))^(1/2)*...
        (log(n_a)/n_a)^(1/2);
        
        
       fprintf("当前有效点数目为:%d\t搜索半径更新为:%0.2f\n",n_a,rn)
    end

%% 采样函数
    function x_sample = Sample(n)
        x_sample = zeros(n,6);
        
        for i = 1:n
            while all(x_sample(i,:)==zeros(1,6))

                if sig
                    % 知情集采样
                    sample = InfromedSampleFree(c_best);
                    if ~isempty(sample)
                        x_sample(i,:) = sample;
                    end
                else
                    % 全体空间采样
                    sample = SampleFree();
                    if ~isempty(sample)
                        x_sample(i,:) = sample;
                    end
                end
            end
                %将采样点加入点列表中
                VA(VAi+1,POS) = x_sample(i,:);
                VAi = VAi + 1;
                Vu.add(VAi)
        end
        n_a  = n_a + n;
        % 更新搜索半径
        Update_rn()
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
        fprintf('当前半径为：%0.2f\n',rn)
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

    end

%% 全范围空间采样-
    function sample = SampleFree()
         sample = zeros(1,6);
         for i = 1:6
            sample(i) = random("Uniform",joint_range{i}(1),joint_range{i}(2));
         end
        % 判断采样点是否有效
        if ~check(sample)
            sample = []; 
        end
    end

%% 在球体内均匀采样
    function sample = Sample_ball()
        sample = sample_in_ndball(zeros(6,1),1,6);
    end

%% 点的有效性检测
function [ret,status] = check(node)
    % 判断是否超出规划空间
    status = 0;
    for i = 1:6
        if node(i) < joint_range{i}(1) || node(i) > joint_range{i}(2)
           ret = false;
           status = 1;
           return
        end
    end
    % 判断是否超出知情集
    if sig ~= 0
        if dist(node-xi') + dist(node-xg') < c_best
            ret = false;
            status = 2;
            return
        end
    end
    % 判断是否发生碰撞
    setconfig(s,node);
    ret = ~checkcollision(s);
    if ~ret
        status = 3;
    end
end
end

