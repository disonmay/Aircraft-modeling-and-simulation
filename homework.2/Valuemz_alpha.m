function output = Valuemz_alpha(input)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
% input为rad
Salpha=[-2 0 2 4 8]*pi/180;
Smz_alpha=[0.02181 0.01961 0.02181 0.02903 0.06310];
if input<=Salpha(1)
    output=Smz_alpha(1);
elseif input>Salpha(1) && input<=Salpha(2)
    output=Smz_alpha(1)+((Smz_alpha(2)-Smz_alpha(1))/(Salpha(2)-Salpha(1)))*(input-Salpha(1));
elseif input>Salpha(2) && input<=Salpha(3)
    output=Smz_alpha(2)+((Smz_alpha(3)-Smz_alpha(2))/(Salpha(3)-Salpha(2)))*(input-Salpha(2));
elseif input>Salpha(3) && input<=Salpha(4)
    output=Smz_alpha(3)+((Smz_alpha(4)-Smz_alpha(3))/(Salpha(4)-Salpha(3)))*(input-Salpha(3));
elseif input>Salpha(4) && input<=Salpha(5)
    output=Smz_alpha(4)+((Smz_alpha(5)-Smz_alpha(4))/(Salpha(5)-Salpha(4)))*(input-Salpha(4));
elseif input>=Salpha(5)
    output=Smz_alpha(5);
end

