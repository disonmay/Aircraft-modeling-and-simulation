function output = ValueCx(input)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
% input为rad
Salpha=[-2 0 2 4 8]*pi/180;
SCx=[0.02181 0.01961 0.02181 0.02903 0.06310];
if input<=Salpha(1)
    output=SCx(1);
elseif input>Salpha(1) && input<=Salpha(2)
    output=SCx(1)+((SCx(2)-SCx(1))/(Salpha(2)-Salpha(1)))*(input-Salpha(1));
elseif input>Salpha(2) && input<=Salpha(3)
    output=SCx(2)+((SCx(3)-SCx(2))/(Salpha(3)-Salpha(2)))*(input-Salpha(2));
elseif input>Salpha(3) && input<=Salpha(4)
    output=SCx(3)+((SCx(4)-SCx(3))/(Salpha(4)-Salpha(3)))*(input-Salpha(3));
elseif input>Salpha(4) && input<=Salpha(5)
    output=SCx(4)+((SCx(5)-SCx(4))/(Salpha(5)-Salpha(4)))*(input-Salpha(4));
elseif input>=Salpha(5)
    output=SCx(5);
end

