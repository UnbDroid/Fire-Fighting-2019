function [tau,Km,delta_mais,delta_menos] = calcula_parametros()
   
    clc

    [delta_menos,delta_mais] = banda_morta();
    delta_menos
    delta_mais
    [tau, Km] = tau_k(delta_menos, delta_mais);
    
end