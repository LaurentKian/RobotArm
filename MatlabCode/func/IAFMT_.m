function [z_r,VA,Con,t,cost,additional_out]=IAFMT_(xi,Xg,s)
%% IAFMT�滮��//���þ��Ȳ���
%--------------------------------------------------------------------------
% ����:  
%       xi: 1*2 double      ���
%       Xg: n*2 double      �յ�����
%       s:  struct          �������


% ���:    
%       z:      double            ·���յ����
%       V:      n*n double        ���ĵ㼯
%       Con:    n*n struct      ���ı߼�

%--------------------------------------------------------------------------
% ��������
eta = 0.2;     %���ڼ���뾶�Ĳ���
rn = 0;        %��ʼ�뾶
n0=50;         %��ʼ����������
n_re=50;       %�����������Ŀ
J_g=0;          %�ɱ���ֵ


Cap=4e4;        %�ڵ�������Ŀ
%--------------------------------------------------------------------------

%% ��ʼ��
NODE = 1;                   % �����������
POS = 2:7;                  % ������������
COST = 8;                   % ·���ɱ�������
PAR = 9;                    % ���ڵ�������
EXPAND = 10;                % ��չ����������
HEUR = 11;                  % ��������ֵ������
START = 1;                  % ���������
GOAL = 2;                   % �յ�������
COL = 10;                   % ���ݾ�������
CHECK_COL = 10;             % ������������ֵ�ж�
N_MAX = 500;                % �ٽ���Ԥ����ռ�


is_heuristic = true;       %�Ƿ��������ʽ�㷨
col = 11;                    %�ڵ��������
check_col = 11;              %�������ɱ��ж�
    
% �ؽڿռ�ķ�Χ
joint_range = s.joint_range;

% �������Ϣ����
% |  1   |  2   -   7  |    8   |   9    |    10    | ... | 11 |
% | Node |    joints   | cost_T | Parent |  Expandi | ... | f  |
VA=NaN*ones(Cap,col);
VA(:,NODE)=1:Cap;  % ���˳��ֵ
VA(:,EXPAND)=0;      % ���е��δ������չ
VAi=0;


a_VA=false(Cap,1); % �ü�����Ч���߼�����
V_un=VA(a_VA);   % ��Ч�����

% ������ӹ�ϵ����
% |  1   |  2   |   3    |  4   | 5  |
% | Node | Near | cost_e | cost | Ni |
near_n = 800; % ��������Ŀ
Con=repmat(struct("Node",NaN,"Near",NaN*ones(near_n,1),"cost_e",...
    NaN*ones(near_n,1),"cost",NaN*ones(near_n,1),'Ni',0),[Cap,1]);


% �����㼯
Vu=Array();
Vo=Array();
Vc=Array();

% �����Ϊ1�ŵ�
Vo.add(START);

if is_heuristic
    % ���������������ֵ��Xg�����ĵ�xg
    xg=Xg;
    VA(START,:)=[1,xi',0,0,0,norm(xi-xg)];
else
    xg=Xg;
    VA(START,:)=[1,xi',0,0,0];
end

VAi=VAi+1;


% �յ����������Ϊ2�ŵ�
% Vu.add(2);
% VA(2,2:3)=Xg(randperm(size(Xg,1),1),:);
% VAi=VAi+1;
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vu.add(GOAL);
VA(GOAL,POS)=xg';
VAi=VAi+1;

Xg=Xg(ceil(size(Xg,1)/2),:);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% ��ʼ�����Ż���flag
sig=0;          

% ��ʼ��Von
Von=Array();

% ��������(������������)
it=0;

% ���ε���ĩ�˲�����
Sample_end=zeros(50,1);

z=1;

% Ŀ������ĳɱ���͵㼰��ɱ�
z_Xg_min=NaN;
c_best = Inf;

% xi - xg �߶��������������ɽǶ�
vector = xg - xi;

% ������յ��ֱ�߾���,��ʼ����Ѿ���
c_min = norm(xi - xg);

% ��Ч����������
n_a = 0;

%% ������
while it<10 
    if sig==0
        Sample(n0);
    
        it=it+1;
        Sample_end(it+1)=VAi;
        a_VA(1:VAi)=1;
        
        
        HybridSearch();
        Eps = 0.01; %�ݲ�
        if VA(z_Xg_min,COST)<=J_g+Eps
            break;
        end
        
    else 
        DynamicOS();
        
        % �ж��յ�ɱ��Ƿ�С���趨��ֵ
        Eps = 0.01; %�ݲ�
        if VA(z_Xg_min,COST)<=J_g+Eps 
            break;
        end
    end
end

z_r=z_Xg_min;
t=toc;
cost=VA(z_r,COST);
if is_heuristic
    fprintf("IAFMT_H:���ս�ɱ�Ϊ %0.2f,��ʱ %0.2f s\n",cost,t)
else
    fprintf("IAFMT:���ս�ɱ�Ϊ %0.2f,��ʱ %0.2f s\n",cost,t)
end

additional_out={VAi,VA};




%% ��Ѱ��ʼ��
    function HybridSearch()

        while isempty(intersect(VA(z,POS),xg','rows'))

            z=ExpandTree(z);
            
            %��ʼ�������޽��򲹳������
            if Vo.num==0 && sig==0  
                InsertNode();
            end
        end
        
        Vc.add(z);
        z_Xg_min=z;

        c_best = VA(z_Xg_min,COST);
        sig=1;
        
        
if is_heuristic
        fprintf("\nIAFMT_H:��ɳ�ʼ������,��ʼ��ɱ�Ϊ%0.2f\n",...
            VA(z_Xg_min,COST))
else
        fprintf("\nIAFMT:��ɳ�ʼ������,��ʼ��ɱ�Ϊ%0.2f\n",...
            VA(z_Xg_min,COST))
end 

end
%% ·���Ż�
    function DynamicOS()

            Prune(VA(z_Xg_min,COST));
            
            Sample(n_re);
           
            it=it+1;
            Sample_end(it+1)=Sample_end(it)+n_re;
            a_VA(Sample_end(it)+1:VAi)=1;

            
            % ���� Vo Vc Vu

            Vo.add(Vc);
            Vo.rm(V_un);
            Vu.rm(V_un);
            Vc.clear();
            z=1;

            if is_heuristic
            % �����������ʽ�㷨�������ɱ����ڵ�ǰ���Ž�ʱ��ֹ����
                while Vo.num~=0 && VA(z,HEUR)<=c_best

                    z=ExpandTree(z);

                    if isempty(z)
                        disp("zΪ��")
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
            fprintf("IAFMT_H:���ε�����ѳɱ�Ϊ%0.2f\n",VA(z_Xg_min,COST))
else
            fprintf("IAFMT:���ε�����ѳɱ�Ϊ%0.2f\n",VA(z_Xg_min,COST))
end

end

%% ����չ
    function z_n=ExpandTree(z)

        % ����Von
        Von.clear();
        
        N_z=Near(z,rn);

        if isempty(N_z)
            X_near=[];
        else
            X_near=N_z(sum(N_z==(Vu.show())',2)>=1);
        end
        
        if ~isempty(X_near)
            for x=X_near'               
                
                % �ҳ�Vo�е�x�ɱ���͵ĵ�
                N_x=Near(x,rn);

                ix=find(sum(N_x==(Vo.show())',2)>=1);
                Y_near=N_x(ix);

                [~,iy]=min(VA(Y_near,COST)+Con(x).cost_e(ix));
                ymin=Y_near(iy);
        
                % y �� x ��Near�����е�����
                x_y_i=Con(x).Near(1:Con(x).Ni)==ymin;
                
                % x �� y ��Near�����е�����
                y_x_i=Con(ymin).Near(1:Con(x).Ni)==x;
                
                if CollF(ymin,x)
                    % �����ӽڵ��ʵ�ʳɱ�
                    Con(x).cost(x_y_i)=Con(x).cost_e(x_y_i);
                    % ����ӽڵ�ĸ��ڵ�
                    VA(x,PAR)=ymin;
                    % ����ӽڵ�ɱ�
                    VA(x,COST)=VA(ymin,COST)+Con(x).cost(x_y_i);
                    if is_heuristic                       
                        % �����ӽڵ����������ֵ
                        VA(x,HEUR)=VA(x,COST)+norm(VA(x,POS)-xg);
                    end
                    
                        
                    % ���¸��ڵ��ʵ�ʳɱ�
                    Con(ymin).cost(y_x_i)=Con(x).cost_e(y_x_i);
                    
                    % ��x��Vu������Von
                    Vu.rm(x);
                    Von.add(x);
                    
                    % ����Ѿ��ҵ���ʼ�⣬�����·���ػ�
                    if sig==1
                        Rewire(Y_near,x);
                    end
                    
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

        % ����z
        [~,z_ni]=min(VA(Vo.show(),check_col));
        Vo_a=Vo.show();
        z_n=Vo_a(z_ni);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%��������㼯�Ƿ���ȷ
if Vo.num+Vc.num+Vu.num ~= sum(a_VA)
    disp("�㼯����")
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
end


%% ���������㲹��
    function InsertNode()
        while 1

            % �������������
            x_s = zeros(1,6);
            for i = 1:6
                x_s(i) = random("Uniform",joint_range{i}(1),joint_range{i}(2));
            end
            vc=Vc.show();
            
            % ����㵽 closed �㼯�ľ���
            Dis=dist(VA(vc,POS)-x_s);
            [dis,vci]=min(Dis);
            
            % �ҵ��������������� closed ��
            vc_c=vc(vci);
            
            if dis>=rn  %����rn��Χ�������steer
                x_n=VA(vc_c,POS)+(x_s-VA(vc_c,POS)*rn/dis);
                dis=norm(VA(vc_c,POS)-x_n);
            else
                x_n=x_s;
            end
            
            if ~check(x_n)
                continue
            end
            
            
            % ��ײ���
            if CollF(vc_c,x_n)

                % ����㼯�б�
                VA(VAi+1,POS)=x_n;
                VA(VAi+1,COST)=VA(vc_c,COST)+dis;
                a_VA(VAi+1)=1;
                VAi=VAi+1;
                
                % Ϊ�²�������Ӹ��ڵ�
                VA(VAi,PAR)=vc_c;
                
                % ���²��������Vo�㼯
                Vo.add(VAi);
                z=VAi;
                
                % Ϊ��������չ�����״̬
                ve=VA(VA(1:VAi,EXPAND)==1,1); 
                dis_c=dist(VA(ve,POS)-VA(VAi,POS));
                
                if any(dis_c==0) % �жϲ������Ƿ���Vc�غ�
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

%% ���ػ�
    function Rewire(Y_near,x)

        
        iy=VA(Y_near,PAR)~=VA(x,PAR);   % ��ȥ���ڵ���ͬ�ĵ�
        H_near=Y_near(iy);
        
        for h=H_near'
            cost_re=VA(x,COST)+dist(VA(h,POS)-VA(x,POS));
            if cost_re<VA(h,COST)
                if CollF(x,h)

                    VA(h,PAR)=x;              % �����ڵ��Ϊx                 
                    sub=VA(h,COST)-cost_re;    
                       
                    % ���¸��ڵ�ĳɱ�
                    VA(h,PAR)=cost_re;
                    if is_heuristic
                        % ���¸��ڵ���������ֵ
                        VA(h,HEUR)=VA(h,COST)+norm(VA(h,POS)-xg);
                    end
                    
                    UpdateChildCosts(h,sub);
                end
            end
        end
    end

%% ��ײ���
    function ret = CollF(x,v)
        x_pos = VA(x,POS);

        if size(v,2)==1         % �ж���������ǵ����Ż��ǵ������
            v_pos=VA(v,POS);
        else
            v_pos=v;
        end
        
        ret = true;
        % �����λ����
        vec = (v_pos - x_pos) / norm(v_pos - x_pos);
        % �������
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

%% �������루�������飩
    function D=dist(V)
        D=sqrt(sum(V.^2,2));
    end
%% Ѱ���ٽ����Ӻ���
function N_z_r=Near(z,rn)

    %�жϸõ��ڵڼ��ε�������չ��������֮�����ĵ�
    Expandi=VA(z,EXPAND);
    if Expandi==it   % �Ѿ��ڸô˵�������չ

       N_z_r=Con(z).Near(1:Con(z).Ni);

    else   
        if VA(z,EXPAND) == 0 %�жϸõ��Ƿ��ѽ��й�����       
           Con(z).Node=z;
        end
        
        if sig==1   %����Ѿ�����֪�鼯��������ȥ���������е���Ч��
            
            i_re1=find(sum(Con(z).Near(1:Con(z).Ni)==V_un',2));
           
           
           % rn ������仯,��ɾ���������о��볬����ǰrn�ĵ�
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
        vi=Sample_end(Expandi+1);       %�ѱ������ĩ��
        VA_a_i=VA(vi+1:VAi,1);          %δ������
        VA_a_i=VA_a_i(a_VA(vi+1:VAi));  %δ����������Ч��
        VA_a=VA(VA_a_i,:);              
        
        
        % ��δ��������ɸѡ����Ч��
        near_dis = dist(VA_a(:,POS)-VA(z,POS));
        N_z_i = near_dis<=rn;
        N_z_i(z) = 0;
        N_z = VA_a(N_z_i,NODE);
        N_size = size(N_z,1);
   
        % ����z��ĸ����ڵ��ϵ
        Con(z).Near(Con(z).Ni+1:Con(z).Ni+N_size)=N_z;
        Con(z).cost_e(Con(z).Ni+1:Con(z).Ni+N_size)=...
            near_dis(N_z_i);
        Con(z).Ni=Con(z).Ni+N_size;
        
        % ����z����չʱ�ĵ�������
        VA(z,EXPAND)=it;
        
        % ���еĸ����ڵ�
        N_z_r=Con(z).Near(1:Con(z).Ni);
        
    end                   
end

%% �ü��� 
    function Prune(cost_g)
        
        % ��Ч������
        ip=dist(VA(1:VAi,POS)-xi')+dist(VA(1:VAi,POS)-VA(z_Xg_min,POS))...
            >cost_g;
        V_un=VA(ip,1);          % ������Ч�����
        a_VA(1:VAi) = ~ip;        % ��Ч������
        VA(V_un,POS) = Inf;   % ������Ч��λ�ã��ɱ�
        VA(V_un,COST) = Inf;
        VA(V_un,PAR)=NaN;     % ɾ����Ч�㸸�ڵ�

        
        % ȥ����Ч��Ϊǰ�̵����Ϣ
        v_un = V_un;
        VA_s = VA(1:VAi,PAR);
        ip_r = v_un;
        while true
            VA(ip_r,PAR)=NaN;
            
            Vo.rm(VA(ip_r,NODE));   % ���ڵ�Ϊ��Ч�����Ƴ�Vo��Vc������Vu
            Vc.rm(VA(ip_r,NODE));
            Vu.add(VA(ip_r,NODE));
            
            ip_r = VA(sum(VA_s==v_un',2)>=1,1);
            v_un = VA(ip_r,NODE);
            if isempty(ip_r)
                break
            end

            VA(ip_r,PAR)=NaN;
            Vo.rm(VA(ip_r,NODE));   % ���ڵ�Ϊ��Ч�����Ƴ�Vo��Vc������Vu
            Vc.rm(VA(ip_r,NODE));
            Vu.add(VA(ip_r,NODE));
        end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 %��鸸�ڵ���Ч��

 V_p=VA(2:VAi,PAR);
 V_p=rmmissing(V_p);
 
 % ���и��ڵ���ڵĽڵ�
 VA_p = VA(V_p,:);

 % �ҳ����ڵ��ǰ�̵���Ϊ�յĵ�
 e_i = find(isnan(VA_p(:,PAR)),1);
 if ~isempty(e_i) 
     fprintf("����ʧЧ���ڵ�:%d\n",VA_p(e_i,1))
 end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        
n_a = sum(~ip);      % ��������Ч����Ŀ
        
end
%% ֪�鼯����-
    function sample_inf = InfromedSampleFree(cost_g)
        % ������ת����C�ͱ��ξ��� L
                x_center = (xi+xg)/2;
                mat = eye(6);
                mat(:,1) = vector;
                C = schmidt_orthogonalization(mat);
                L = eye(6)*sqrt(cost_g^2-c_min^2)/2;
                L(1,1) = cost_g/2;
                
                % ��λ���ھ��Ȳ�������Ҫ��Ϊ����������
                sample_inf = C*L*Sample_ball() + x_center; 
                sample_inf = sample_inf';
        
        % �жϲ������Ƿ���Ч
        if ~check(sample_inf)
            sample_inf = []; 
        end
    end
%% �����ӽڵ�ɱ�
    function UpdateChildCosts(h,sub)
        vo=Vo.show();
        c_size=500;
        Children=zeros(c_size,1);   %����ӽڵ�Ķ���
        Ci1=0;                      %�ӽڵ������ָ��
        
        vo_add=vo(VA(vo,PAR)==h);      % Ѱ�Ҹ��ڵ�Ϊh�ĵ�
        vo(VA(vo,PAR)==h)=[];          % ȥ���Ѽ����ڵ�
        
        if ~isempty(vo_add)
            VA(vo_add,COST)=VA(vo_add,COST)-sub;  % ���½ڵ�ɱ�
            
            Children(Ci1+1:Ci1+size(vo_add,1))=vo_add;
            Ci2=Ci1+size(vo_add,1);         %�ӽڵ������ָ��
            
            while 1
                % ��Ѱ�ӽڵ�
                ip=sum(VA(vo,PAR)==(Children(Ci1+1:Ci2))',2)>=1;  
                if any(ip)
                    vo_add=vo(ip);
                    vo(ip)=[]; % ���Ѿ��������ӽڵ㣬�Ӷ�����ȥ��
                    VA(vo_add,COST)=VA(vo_add,COST)-sub;  % ���½ڵ�ɱ�
                    
                    % ���½ڵ���������ֵ
                    if is_heuristic
                        VA(vo_add,HEUR)=VA(vo_add,COST)+dist(VA(vo_add,POS)-xg);
                    end
                    Children(Ci2+1:Ci2+size(vo_add,1))=vo_add;
                    
                    Ci1=Ci2;               % �����ӽڵ������βָ��
                    Ci2=Ci2+size(vo_add,1);
                else
                    break;
                end
            end
        end
    end
%% ���������뾶
    function Update_rn()
        
        % ȫ������ռ�Ĳ��
        X_A = s.cspace_lebes;
        % ֪�鼯�Ĳ��
        a = c_best/2; % ����
        b = sqrt(c_best^2 - norm(xi-xg)^2)/2; %  ����
        X_inf = (2^6*a*b^5*gamma(1/6)^6)/(6^6*gamma(2));
        % ��������뾶�����
        X = min(X_A,X_inf);
        
        rn = 5*(1+eta)*2*(1/2)^(1/2)*(X/(pi))^(1/2)*...
        (log(n_a)/n_a)^(1/2);
        
        
       fprintf("��ǰ��Ч����ĿΪ:%d\t�����뾶����Ϊ:%0.2f\n",n_a,rn)
    end

%% ��������
    function x_sample = Sample(n)
        x_sample = zeros(n,6);
        
        for i = 1:n
            while all(x_sample(i,:)==zeros(1,6))

                if sig
                    % ֪�鼯����
                    sample = InfromedSampleFree(c_best);
                    if ~isempty(sample)
                        x_sample(i,:) = sample;
                    end
                else
                    % ȫ��ռ����
                    sample = SampleFree();
                    if ~isempty(sample)
                        x_sample(i,:) = sample;
                    end
                end
            end
                %�������������б���
                VA(VAi+1,POS) = x_sample(i,:);
                VAi = VAi + 1;
                Vu.add(VAi)
        end
        n_a  = n_a + n;
        % ���������뾶
        Update_rn()
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
        fprintf('��ǰ�뾶Ϊ��%0.2f\n',rn)
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

    end

%% ȫ��Χ�ռ����-
    function sample = SampleFree()
         sample = zeros(1,6);
         for i = 1:6
            sample(i) = random("Uniform",joint_range{i}(1),joint_range{i}(2));
         end
        % �жϲ������Ƿ���Ч
        if ~check(sample)
            sample = []; 
        end
    end

%% �������ھ��Ȳ���
    function sample = Sample_ball()
        sample = sample_in_ndball(zeros(6,1),1,6);
    end

%% �����Ч�Լ��
function [ret,status] = check(node)
    % �ж��Ƿ񳬳��滮�ռ�
    status = 0;
    for i = 1:6
        if node(i) < joint_range{i}(1) || node(i) > joint_range{i}(2)
           ret = false;
           status = 1;
           return
        end
    end
    % �ж��Ƿ񳬳�֪�鼯
    if sig ~= 0
        if dist(node-xi') + dist(node-xg') < c_best
            ret = false;
            status = 2;
            return
        end
    end
    % �ж��Ƿ�����ײ
    setconfig(s,node);
    ret = ~checkcollision(s);
    if ~ret
        status = 3;
    end
end
end

