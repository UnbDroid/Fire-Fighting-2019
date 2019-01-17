function plot_step(C)
% Plota a resposta ao degrau para um sistema de controle com os
% parâmetros PID contidos em C para o processo identificado
% no laboratório (planta linear).

[kp,ki,kd] = calcula_constantes_PID(C);
g = tf([179.4584],[0.1733 1 0]);
c = tf([kd kp ki],[1 0]);

closedloop = feedback(g*c,1);

[Y,T] = step(closedloop);
plot(T,Y)


end