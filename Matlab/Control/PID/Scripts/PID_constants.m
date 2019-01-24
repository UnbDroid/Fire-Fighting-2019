function 
    % A partir da função de transferencia do controlador, calcula e seta no
    % workspace as variáveis kp,ki e kd do PID para utilizar no Simulink.

    clc
    
    [num_l, den_l] = tfdata(C_l);
    cell2mat(num_l);
    n_l = num_l{1,1};
    
    kd_l = n_l(1);
    kp_l = n_l(2);
    ki_l = n_l(3);
    
    [num_r, den_r] = tfdata(C_r);
    cell2mat(num_r);
    n_r = num_r{1,1};
    
    kd_r = n_r(1);
    kp_r = n_r(2);
    ki_r = n_r(3);
    
end