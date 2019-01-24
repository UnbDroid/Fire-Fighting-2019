[delta_negative_l,delta_positive_l,delta_negative_r,delta_positive_r] = Dead_Zone();
[tau_l,K_l,tau_r,K_r] = Transfer_Function(delta_negative_l, delta_positive_l,delta_negative_r, delta_positive_r);

