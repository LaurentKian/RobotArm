function [z,VA,Con,t,cost,additional_out]=FMT_with_weight(xi,Xg,samples,s)
%% FMT规划器(用于机械臂)
%--------------------------------------------------------------------------
% 输入:  
%       xi:         1*2 double      起点
%       Xg:         n*2 double      终点区域
%       samples:    k*n double      采样点
%       s:          struct          仿真设置
            
% 输出:    
%       sequence:   k*n' double     轨迹序列
%--------------------------------------------------------------------------

% 参数设置

eta = 0.2;      %用于计算半径的参数
R = [1.6,1.2,1.0,0.8,0.6,0.4];

%--------------------------------------------------------------------------

%% 初始化

NODE = 1;                   % 点序号所在列
POS = 2:7;                  % 点坐标所在列
COST = 8;                   % 路径成本所在列
PAR = 9;                    % 父节点所在列
HEUR = 10;                  % 启发函数值所在列
START = 1;                  % 起点所在行
GOAL = 2;                   % 终点所在行
COL = 10;                   % 数据矩阵列数
CHECK_COL = 10;             % 依据启发函数值判断
N_MAX = 500;                % 临近点预分配空间


%搜索半径
n = size(samples,2)+2; %采样点数目
d = 6; % 空间维度
rn = (1+eta)*2*(1/d)^(1/d)*(s.cspace_lebes/(pi^3/6))^(1/d)*(log(n)/n)^(1/d);

fprintf("搜索半径为:%0.2f\n",rn)

% 点基本信息矩阵
% |  1   |  2   -   7  |   8    |   9    | 10 |
% | Node |    joints   | cost_T | Parent |  f |

VA=NaN*ones(n,COL);
    
VA(:,NODE)=1:n;  % 序号顺序赋值

% 点的连接关系矩阵
% |  1   |  2   |   3    |  4   | 5  |
% | Node | Near | cost_e | cost | Ni |
Con=repmat(struct("Node",NaN,"Near",NaN*ones(N_MAX,1),"cost_e",...
    NaN*ones(N_MAX,1),"cost",NaN*ones(N_MAX,1),'Ni',0),[n,1]);


% 三个点集
Vu=Array();
Vo=Array();
Vc=Array();

% 起点作为序号为1的点
Vo.add(START);

VA(START,NODE) = 1;
VA(START,POS) = xi';
VA(START,COST) = 0;
VA(START,PAR) = 0;
VA(START,HEUR) = norm(xi-Xg);

% 终点区域随机点为2号点

VA(GOAL,POS)=Xg';
Vu.add(GOAL);

% 将其他采样点加入数据矩阵
n_samples = size(samples,2);
VA(3:3+n_samples-1,POS) = samples';
Vu.add(3:3+n_samples-1);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% plot(VA(:,2),VA(:,3),'.')
% VA0=VA;
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% 初始化Von
Von=Array();

z=START;

%% 主程序
tic
while isempty(intersect(VA(z,POS),Xg','rows'))
    
    z=ExpandTree(z);
        
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% plot(VA(z,2),VA(z,3),'ro','MarkerFaceColor','red')
% plot(VA(Vc.show(),2),VA(Vc.show(),3),'ro','MarkerFaceColor','black')
% plot(VA(Vo.show(),2),VA(Vo.show(),3),'c*')
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
end

t=toc;
cost=VA(z,COST);

msg='\nFMT_H:找到可行解,用时 %0.2f s,路径成本%0.2f\n';

fprintf(msg,t,cost);
additional_out = [];

%% 寻找临近点
function N_x=Near(x,rn)
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%        disp('执行Near函数')
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    if Con(x).Ni==0 %判断该点是否已进行过运算
       Con(x).Node=x;
        
        near_dis = dist(VA(:,POS)-VA(x,POS));
        N_x_i = near_dis<=rn;
        N_x_i(x) = 0;
        N_x = VA(N_x_i,NODE);
        N_size = size(N_x,1);
        
        % 更新z点的附近节点关系
        Con(x).Near(1:N_size)=N_x;
        Con(x).cost_e(1:N_size)=near_dis(N_x_i);
        Con(x).Ni=N_size;
    else
        N_x=Con(x).Near(1:Con(x).Ni);
    end
end


%% 碰撞检测
    function ret = collfree(x,v)
        x_pos = VA(x,POS);
        v_pos = VA(v,POS);
        dl = deg2rad(2);

        ret = edgecheck(x_pos,v_pos,dl,s);
    end

%% 两点间距离（用于数组）
    function D=dist(V)
        D=sqrt(sum((R.*V).^2,2));
    end

%% 树拓展
    function z_n=ExpandTree(z)
        % 重置Von
        Von.clear();
        
        N_z=Near(z,rn);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% plot(VA(N_z,2),VA(N_z,3),'b.')
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

%       X_near=intersect(N_z,Vu.show());
        X_near=N_z(sum(N_z==Vu.show()',2) >=1);
        
        if ~isempty(X_near)
            for x=X_near'
                
                % 找出Vo中到x成本最低的点
                N_x=Near(x,rn);
                
                ix=sum(N_x==Vo.show()',2) >=1;
                
                Y_near=N_x(ix);
                [~,iy]=min(VA(Y_near,COST)+Con(x).cost_e(ix));
                ymin=Y_near(iy);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~               
%                 [Y_near,ix]=intersect(N_x,Vo.show(),'rows');
%                 [~,iy]=min(VA(Y_near,4)+Con(x).cost_e(ix));
%                 ymin=Y_near(iy);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~         
                % y 在 x 的Near队列中的索引
                x_y_i=Con(x).Near(1:Con(x).Ni)==ymin;
                
                % x 在 y 的Near队列中的索引
                y_x_i=Con(ymin).Near(1:Con(x).Ni)==x;
                
                if collfree(ymin,x)
               
                    % 更新子节点的实际成本
                    Con(x).cost(x_y_i)=Con(x).cost_e(x_y_i);
                    % 添加子节点的父节点
                    VA(x,PAR)=ymin;
                    % 添加子节点成本
                    VA(x,COST)=VA(ymin,COST)+Con(x).cost(x_y_i);
                    % 更新子节点的启发函数值
                    VA(x,HEUR)=VA(x,COST)+norm(VA(x,POS)-Xg);
                    
                        
                    % 更新父节点的实际成本
                    Con(ymin).cost(y_x_i)=Con(x).cost_e(y_x_i);
                    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% plot([VA(x,2),VA(ymin,2)],[VA(x,3),VA(ymin,3)],'r-')
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    
                    % 将x从Vu移入入Von
                    Vu.rm(x);
                    Von.add(x);                    
                    
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

        
        if Vo.num==0   
            error('初始解搜索失败！');
        end
         

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%         disp("进入下一次迭代")
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        % 更新z
        [~,z_ni]=min(VA(Vo.show(),CHECK_COL));
        Vo_a=Vo.show();
        z_n=Vo_a(z_ni);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%检查三个点集是否正确
% if Vo.num+Vc.num+Vu.num ~= n && isempty(intersect(Vo,show(),Vc.show())) ...
%         && isempty(intersect(Vo,show(),Vu.show())) ...
%         && isempty(intersect(Vu,show(),Vc.show()))
%     disp("点集错误")
% end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    end
end

            
            
        

