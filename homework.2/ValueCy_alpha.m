function output = ValueCy_alpha(input)
%UNTITLED2 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
% inputΪrad
Salpha=[-2 0 2 4 8]*pi/180;
SCy_alpha=[-0.0666 0 0.0670 0.1479 0.3540];
if input<=Salpha(1)
    output=SCy_alpha(1);
elseif input>Salpha(1) && input<=Salpha(2)
    output=SCy_alpha(1)+((SCy_alpha(2)-SCy_alpha(1))/(Salpha(2)-Salpha(1)))*(input-Salpha(1));
elseif input>Salpha(2) && input<=Salpha(3)
    output=SCy_alpha(2)+((SCy_alpha(3)-SCy_alpha(2))/(Salpha(3)-Salpha(2)))*(input-Salpha(2));
elseif input>Salpha(3) && input<=Salpha(4)
    output=SCy_alpha(3)+((SCy_alpha(4)-SCy_alpha(3))/(Salpha(4)-Salpha(3)))*(input-Salpha(3));
elseif input>Salpha(4) && input<=Salpha(5)
    output=SCy_alpha(4)+((SCy_alpha(5)-SCy_alpha(4))/(Salpha(5)-Salpha(4)))*(input-Salpha(4));
elseif input>=Salpha(5)
    output=SCy_alpha(5);
end

