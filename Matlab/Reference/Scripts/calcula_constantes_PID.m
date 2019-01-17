function [kp,ki,kd] = calcula_constantes_PID(C)
    % A partir da função de transferencia do controlador, calcula e seta no
    % workspace as variáveis kp,ki e kd do PID para utilizar no Simulink.

    clc
    
    [num, den] = tfdata(C);
    cell2mat(num);
    n = num{1,1};
    
    kd = n(1);
    kp = n(2);
    ki = n(3);

end