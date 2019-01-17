%%Given the 3 tables imported to matlab, this script generate all values to be used in the identification. 

time1 = Square1{1:end,1};
time2 = Square2{1:end,1};
time3 = Triangular{1:end,1};
time1 = time1/1000;
time2 = time2/1000;
time3 = time3/1000;
time1 = time1.';
time2 = time2.';
time3 = time3.';

pwm1 = Square1{1:end,2};
pwm2 = Square2{1:end,2};
pwm3 = Triangular{1:end,2};
pwm1 = pwm1.';
pwm2 = pwm2.';
pwm3 = pwm3.';

encoder_left1 = Square1{1:end,3};
encoder_left2 = Square2{1:end,3};
encoder_left3 = Triangular{1:end,3};
encoder_left1 = encoder_left1.';
encoder_left2 = encoder_left2.';
encoder_left3 = encoder_left3.';
speed_left1 = diff(encoder_left1);
speed_left2 = diff(encoder_left2);
speed_left3 = diff(encoder_left3);
speed_left1(length(time1)) = speed_left1(length(time1)-1);
speed_left2(length(time2)) = speed_left2(length(time2)-1);
speed_left3(length(time3)) = speed_left3(length(time3)-1);

encoder_right1 = Square1{1:end,4};
encoder_right2 = Square2{1:end,4};
encoder_right3 = Triangular{1:end,4};
encoder_right1 = encoder_right1.';
encoder_right2 = encoder_right2.';
encoder_right3 = encoder_right3.';
speed_right1 = diff(encoder_right1);
speed_right2 = diff(encoder_right2);
speed_right3 = diff(encoder_right3);
speed_right1(length(time1)) = speed_right1(length(time1)-1);
speed_right2(length(time2)) = speed_right2(length(time2)-1);
speed_right3(length(time3)) = speed_right3(length(time3)-1);
