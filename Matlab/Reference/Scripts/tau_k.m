function [tau,K] = tau_k(delta_menos, delta_mais)
    clc
    
    dados_linear = load('dados_quad2.mat');
    todos_dados = dados_linear.dados;
    t_sampling = 0.002;
    tamanho = length(todos_dados);

    for i = 1:tamanho

        w(i) = todos_dados(2,i);
    end

    for i = 1:tamanho

        if todos_dados(4,i) > delta_menos && todos_dados(4,i) < delta_mais

            u(i) = 0;
        elseif todos_dados(4,i) <= delta_menos

            u(i) = todos_dados(4,i) - delta_menos;
        else 

            u(i) = todos_dados(4,i) - delta_mais;
        end
    end

    for i = 1:(tamanho-1)

        Y(i,1) = w(i+1);
        phi(i, 1) = w(i);
        phi(i, 2) = u(i+1); 
    end

    theta = pinv(phi'*phi)*phi'*Y;

    a = theta(1,1);
    b = theta(2,1);

    tau = t_sampling/(1/a -1);
    K = b*(tau + t_sampling)/t_sampling;    
end