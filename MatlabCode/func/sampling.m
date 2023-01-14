function samples = sampling(s,n)
%% 在 C-space 采样有效点
% param: s  仿真设置             | struct      
%        n  总采样点数目         | 1*1 double     
%--------------------------------------------------------------------------
% return: samples 有效采样点     | n*n double
%--------------------------------------------------------------------------

    samples_all = sampleC(s,n);
    collision = false(1,size(samples_all,2));
    for i = 1:size(samples_all,2)
        setconfig(s,samples_all(:,i));
        ret = checkcollision(s);
        if ret
            collision(i) = true;
        end
    end
    samples = samples_all(:,~collision);
end