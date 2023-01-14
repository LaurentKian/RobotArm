function [z,VA,Con,t,cost,additional_out]=FMT_with_weight(xi,Xg,samples,s)
%% FMT�滮��(���ڻ�е��)
%--------------------------------------------------------------------------
% ����:  
%       xi:         1*2 double      ���
%       Xg:         n*2 double      �յ�����
%       samples:    k*n double      ������
%       s:          struct          ��������
            
% ���:    
%       sequence:   k*n' double     �켣����
%--------------------------------------------------------------------------

% ��������

eta = 0.2;      %���ڼ���뾶�Ĳ���
R = [1.6,1.2,1.0,0.8,0.6,0.4];

%--------------------------------------------------------------------------

%% ��ʼ��

NODE = 1;                   % �����������
POS = 2:7;                  % ������������
COST = 8;                   % ·���ɱ�������
PAR = 9;                    % ���ڵ�������
HEUR = 10;                  % ��������ֵ������
START = 1;                  % ���������
GOAL = 2;                   % �յ�������
COL = 10;                   % ���ݾ�������
CHECK_COL = 10;             % ������������ֵ�ж�
N_MAX = 500;                % �ٽ���Ԥ����ռ�


%�����뾶
n = size(samples,2)+2; %��������Ŀ
d = 6; % �ռ�ά��
rn = (1+eta)*2*(1/d)^(1/d)*(s.cspace_lebes/(pi^3/6))^(1/d)*(log(n)/n)^(1/d);

fprintf("�����뾶Ϊ:%0.2f\n",rn)

% �������Ϣ����
% |  1   |  2   -   7  |   8    |   9    | 10 |
% | Node |    joints   | cost_T | Parent |  f |

VA=NaN*ones(n,COL);
    
VA(:,NODE)=1:n;  % ���˳��ֵ

% ������ӹ�ϵ����
% |  1   |  2   |   3    |  4   | 5  |
% | Node | Near | cost_e | cost | Ni |
Con=repmat(struct("Node",NaN,"Near",NaN*ones(N_MAX,1),"cost_e",...
    NaN*ones(N_MAX,1),"cost",NaN*ones(N_MAX,1),'Ni',0),[n,1]);


% �����㼯
Vu=Array();
Vo=Array();
Vc=Array();

% �����Ϊ���Ϊ1�ĵ�
Vo.add(START);

VA(START,NODE) = 1;
VA(START,POS) = xi';
VA(START,COST) = 0;
VA(START,PAR) = 0;
VA(START,HEUR) = norm(xi-Xg);

% �յ����������Ϊ2�ŵ�

VA(GOAL,POS)=Xg';
Vu.add(GOAL);

% ������������������ݾ���
n_samples = size(samples,2);
VA(3:3+n_samples-1,POS) = samples';
Vu.add(3:3+n_samples-1);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% plot(VA(:,2),VA(:,3),'.')
% VA0=VA;
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% ��ʼ��Von
Von=Array();

z=START;

%% ������
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

msg='\nFMT_H:�ҵ����н�,��ʱ %0.2f s,·���ɱ�%0.2f\n';

fprintf(msg,t,cost);
additional_out = [];

%% Ѱ���ٽ���
function N_x=Near(x,rn)
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%        disp('ִ��Near����')
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    if Con(x).Ni==0 %�жϸõ��Ƿ��ѽ��й�����
       Con(x).Node=x;
        
        near_dis = dist(VA(:,POS)-VA(x,POS));
        N_x_i = near_dis<=rn;
        N_x_i(x) = 0;
        N_x = VA(N_x_i,NODE);
        N_size = size(N_x,1);
        
        % ����z��ĸ����ڵ��ϵ
        Con(x).Near(1:N_size)=N_x;
        Con(x).cost_e(1:N_size)=near_dis(N_x_i);
        Con(x).Ni=N_size;
    else
        N_x=Con(x).Near(1:Con(x).Ni);
    end
end


%% ��ײ���
    function ret = collfree(x,v)
        x_pos = VA(x,POS);
        v_pos = VA(v,POS);
        dl = deg2rad(2);

        ret = edgecheck(x_pos,v_pos,dl,s);
    end

%% �������루�������飩
    function D=dist(V)
        D=sqrt(sum((R.*V).^2,2));
    end

%% ����չ
    function z_n=ExpandTree(z)
        % ����Von
        Von.clear();
        
        N_z=Near(z,rn);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% plot(VA(N_z,2),VA(N_z,3),'b.')
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

%       X_near=intersect(N_z,Vu.show());
        X_near=N_z(sum(N_z==Vu.show()',2) >=1);
        
        if ~isempty(X_near)
            for x=X_near'
                
                % �ҳ�Vo�е�x�ɱ���͵ĵ�
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
                % y �� x ��Near�����е�����
                x_y_i=Con(x).Near(1:Con(x).Ni)==ymin;
                
                % x �� y ��Near�����е�����
                y_x_i=Con(ymin).Near(1:Con(x).Ni)==x;
                
                if collfree(ymin,x)
               
                    % �����ӽڵ��ʵ�ʳɱ�
                    Con(x).cost(x_y_i)=Con(x).cost_e(x_y_i);
                    % ����ӽڵ�ĸ��ڵ�
                    VA(x,PAR)=ymin;
                    % ����ӽڵ�ɱ�
                    VA(x,COST)=VA(ymin,COST)+Con(x).cost(x_y_i);
                    % �����ӽڵ����������ֵ
                    VA(x,HEUR)=VA(x,COST)+norm(VA(x,POS)-Xg);
                    
                        
                    % ���¸��ڵ��ʵ�ʳɱ�
                    Con(ymin).cost(y_x_i)=Con(x).cost_e(y_x_i);
                    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% plot([VA(x,2),VA(ymin,2)],[VA(x,3),VA(ymin,3)],'r-')
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    
                    % ��x��Vu������Von
                    Vu.rm(x);
                    Von.add(x);                    
                    
                else
                    Con(x).cost(x_y_i)=Inf;
                    Con(ymin).cost(y_x_i)=Inf;
                end
            end      
        end
        % ����Vo��Vc
        Vo.rm(z);
        Vc.add(z);
        Vo.add(Von);

        
        if Vo.num==0   
            error('��ʼ������ʧ�ܣ�');
        end
         

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%         disp("������һ�ε���")
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        % ����z
        [~,z_ni]=min(VA(Vo.show(),CHECK_COL));
        Vo_a=Vo.show();
        z_n=Vo_a(z_ni);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%��������㼯�Ƿ���ȷ
% if Vo.num+Vc.num+Vu.num ~= n && isempty(intersect(Vo,show(),Vc.show())) ...
%         && isempty(intersect(Vo,show(),Vu.show())) ...
%         && isempty(intersect(Vu,show(),Vc.show()))
%     disp("�㼯����")
% end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    end
end

            
            
        

