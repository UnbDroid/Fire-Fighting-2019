function [tau_l,K_l,tau_r,K_r] = Transfer_Function(delta_negative_l, delta_positive_l,delta_negative_r, delta_positive_r)
    clc
    
    load('Data1.mat');
    CurvesSetup;
    t_sampling = 0.002;
    size = length(pwm1);
        
    for i = 1:size

        w_l(i) = speed_left1(1,i);
        w_r(i) = speed_right1(1,i);
        
    end

    for i = 1:size
        
        if pwm1(1,i) > delta_negative_l && pwm1(1,i) < delta_positive_l
            u_l(i) = 0;
        elseif pwm1(1,i) <= delta_negative_l

            u_l(i) = pwm1(1,i) - delta_negative_l;
        else 

            u_l(i) = pwm1(1,i) - delta_positive_l;
        end
        
        if pwm1(1,i) > delta_negative_r && pwm1(1,i) < delta_positive_r
            u_r(i) = 0;
        elseif pwm1(1,i) <= delta_negative_r

            u_r(i) = pwm1(1,i) - delta_negative_r;
        else 

            u_r(i) = pwm1(1,i) - delta_positive_r;
        end
    end
    

    for i = 1:(size-1)
        Y_r(i,1) = w_r(i+1);
        phi_r(i, 1) = w_r(i);
        phi_r(i, 2) = u_r(i+1);
        
        Y_l(i,1) = w_l(i+1);
        phi_l(i, 1) = w_l(i);
        phi_l(i, 2) = u_l(i+1); 
    end

    theta_r = pinv(phi_r'*phi_r)*phi_r'*Y_r;

    a_r = theta_r(1,1);
    b_r = theta_r(2,1);

    tau_r = t_sampling/(1/a_r -1);
    K_r = b_r*(tau_r + t_sampling)/t_sampling;    
    
    theta_l = pinv(phi_l'*phi_l)*phi_l'*Y_l;

    a_l = theta_l(1,1);
    b_l = theta_l(2,1);

    tau_l = t_sampling/(1/a_l -1);
    K_l = b_l*(tau_l + t_sampling)/t_sampling;    
end