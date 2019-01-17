function [delta_menos,delta_mais] = banda_morta()

    dados_linear = load('dados_triang.mat');
    todos_dados = dados_linear.dados;

    somatorio_pos = 0;
    somatorio_neg = 0;
    aux_neg = 0;
    aux_pos = 0;
    aux_total = 0;

    for i = 1:length(todos_dados)
        if abs(todos_dados(2,i)) < 3
            todos_dados(6,i) = 0;
        elseif todos_dados(2,i) > 0
            todos_dados(6,i) = 1;
        else 
            todos_dados(6,i) = -1;
        end
    end
    for i = 2:length(todos_dados)
        if todos_dados(6,i) ~= 0
            if todos_dados(6,i-1) - todos_dados(6,i) < 0 
                somatorio_pos = somatorio_pos + todos_dados(5,i);
                aux_pos = aux_pos +1;
            elseif todos_dados(6,i-1) - todos_dados(6,i) > 0
                somatorio_neg = somatorio_neg + todos_dados(5,i);
                aux_neg = aux_neg + 1;
            end 
        end
    end
    
    delta_menos = somatorio_neg / aux_neg;
    delta_mais = somatorio_pos / aux_pos;
    
end