function samples = sampleC(s,n)
%% 在 C-space 进行采样
% param: s  仿真设置       | struct        
%        n  采样点数目   | 1*1 double  
%--------------------------------------------------------------------------
% return: samples 采样点   | n*n double
    row = size(s.joint_range,2);
    samples = zeros(row,n);
    for k = 1:row
        samples(k,:) = random("Uniform",s.joint_range{k}(1), ...
            s.joint_range{k}(2),1,n);
    end
end