function [delta_negative_l,delta_positive_l,delta_negative_r,delta_positive_r] = Dead_Zone()
    clc
    load('Data1.mat');
    CurvesSetup;

    sum_pos_l = 0;
    sum_neg_l = 0;
    sum_pos_r = 0;
    sum_neg_r = 0;
    
    for i = 1:length(time3);
        if abs(speed_left3(1,i)) == 0
            speed_left3(2,i) = 0;
        elseif speed_left3(1,i) > 0
            speed_left3(2,i) = 1;
        else 
            speed_left3(2,i) = -1;
        end
    end
    
    for i = 1:length(time3);
        if abs(speed_right3(1,i)) == 0
            speed_right3(2,i) = 0;
        elseif speed_right3(1,i) > 0
            speed_right3(2,i) = 1;
        else 
            speed_right3(2,i) = -1;
        end 
    end
    
    aux_neg = 0;
    aux_pos = 0;

    for i = 2:length(time3)
        if speed_right3(2,i) ~= 0
            if speed_right3(2,i-1) - speed_right3(2,i) < 0 
                sum_pos_r = sum_pos_r + pwm3(1,i);
                aux_pos = aux_pos +1;
            elseif speed_right3(2,i-1) - speed_right3(2,i) > 0
                sum_neg_r = sum_neg_r + pwm3(1,i);
                aux_neg = aux_neg + 1;
            end 
        end
    end
    
    aux_neg = 0;
    aux_pos = 0;
    
    for i = 2:length(time3)
        if speed_left3(2,i) ~= 0
            if speed_left3(2,i-1) - speed_left3(2,i) < 0 
                sum_pos_l = sum_pos_l + pwm3(1,i);
                aux_pos = aux_pos +1;
            elseif speed_left3(2,i-1) - speed_left3(2,i) > 0
                sum_neg_l = sum_neg_l + pwm3(1,i);
                aux_neg = aux_neg + 1;
            end 
        end
    end
    
    delta_negative_l = sum_neg_l / aux_neg;
    delta_positive_l = sum_pos_l / aux_pos;
    delta_negative_r = sum_neg_r / aux_neg;
    delta_positive_r = sum_pos_r / aux_pos;
    
end