function orth_mat=schmidt_orthogonalization(mat)
%% 斯密特正交化
% param: mat  无关向量组         | m*n double           
%--------------------------------------------------------------------------
% return: orth_mat 正交向量组    | m*n double
%--------------------------------------------------------------------------

[m,n] = size(mat);
if(m<n)
     warning('行小于列，无法计算，请转置后重新输入');
     return
end
orth_mat=zeros(m,n);

%正交化
orth_mat(:,1)=mat(:,1);
for i=2:n
 for j=1:i-1
     orth_mat(:,i)=orth_mat(:,i)-dot(mat(:,i),orth_mat(:,j))/dot(orth_mat(:,j),orth_mat(:,j))*orth_mat(:,j);
 end
 orth_mat(:,i)=orth_mat(:,i)+mat(:,i);
end

% 单位化
for k=1:n
    orth_mat(:,k)=orth_mat(:,k)/norm(orth_mat(:,k));
end