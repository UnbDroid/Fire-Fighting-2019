[tau,Km,delta_mais,delta_menos] = calcula_parametros();
G = tf([Km],[tau 1 0]);
sisotool(G)
% Depois de importar o C do sisotool
[kp,ki,kd] = calcula_constantes_PID(C);