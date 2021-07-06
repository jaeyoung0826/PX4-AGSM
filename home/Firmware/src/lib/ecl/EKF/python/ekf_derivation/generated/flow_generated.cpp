// X Axis Equations
// Sub Expressions
const float HK0 = -Tbs(1,0)*q2 + Tbs(1,1)*q1 + Tbs(1,2)*q0;
const float HK1 = Tbs(1,0)*q3 + Tbs(1,1)*q0 - Tbs(1,2)*q1;
const float HK2 = Tbs(1,0)*q0 - Tbs(1,1)*q3 + Tbs(1,2)*q2;
const float HK3 = HK0*vd + HK1*ve + HK2*vn;
const float HK4 = 1.0F/range;
const float HK5 = 2*HK4;
const float HK6 = Tbs(1,0)*q1 + Tbs(1,1)*q2 + Tbs(1,2)*q3;
const float HK7 = -HK0*ve + HK1*vd + HK6*vn;
const float HK8 = HK0*vn - HK2*vd + HK6*ve;
const float HK9 = -HK1*vn + HK2*ve + HK6*vd;
const float HK10 = q0*q2;
const float HK11 = q1*q3;
const float HK12 = 2*Tbs(1,2);
const float HK13 = q0*q3;
const float HK14 = q1*q2;
const float HK15 = 2*Tbs(1,1);
const float HK16 = powf(q1, 2);
const float HK17 = powf(q2, 2);
const float HK18 = -HK17;
const float HK19 = powf(q0, 2);
const float HK20 = powf(q3, 2);
const float HK21 = HK19 - HK20;
const float HK22 = HK12*(HK10 + HK11) - HK15*(HK13 - HK14) + Tbs(1,0)*(HK16 + HK18 + HK21);
const float HK23 = 2*Tbs(1,0);
const float HK24 = q0*q1;
const float HK25 = q2*q3;
const float HK26 = -HK16;
const float HK27 = -HK12*(HK24 - HK25) + HK23*(HK13 + HK14) + Tbs(1,1)*(HK17 + HK21 + HK26);
const float HK28 = HK15*(HK24 + HK25) - HK23*(HK10 - HK11) + Tbs(1,2)*(HK18 + HK19 + HK20 + HK26);
const float HK29 = 2*HK3;
const float HK30 = 2*HK7;
const float HK31 = 2*HK8;
const float HK32 = 2*HK9;
const float HK33 = HK22*P(0,4) + HK27*P(0,5) + HK28*P(0,6) + HK29*P(0,0) + HK30*P(0,1) + HK31*P(0,2) + HK32*P(0,3);
const float HK34 = powf(range, -2);
const float HK35 = HK22*P(4,6) + HK27*P(5,6) + HK28*P(6,6) + HK29*P(0,6) + HK30*P(1,6) + HK31*P(2,6) + HK32*P(3,6);
const float HK36 = HK22*P(4,5) + HK27*P(5,5) + HK28*P(5,6) + HK29*P(0,5) + HK30*P(1,5) + HK31*P(2,5) + HK32*P(3,5);
const float HK37 = HK22*P(4,4) + HK27*P(4,5) + HK28*P(4,6) + HK29*P(0,4) + HK30*P(1,4) + HK31*P(2,4) + HK32*P(3,4);
const float HK38 = HK22*P(2,4) + HK27*P(2,5) + HK28*P(2,6) + HK29*P(0,2) + HK30*P(1,2) + HK31*P(2,2) + HK32*P(2,3);
const float HK39 = HK22*P(3,4) + HK27*P(3,5) + HK28*P(3,6) + HK29*P(0,3) + HK30*P(1,3) + HK31*P(2,3) + HK32*P(3,3);
const float HK40 = HK22*P(1,4) + HK27*P(1,5) + HK28*P(1,6) + HK29*P(0,1) + HK30*P(1,1) + HK31*P(1,2) + HK32*P(1,3);
const float HK41 = HK4/(HK22*HK34*HK37 + HK27*HK34*HK36 + HK28*HK34*HK35 + HK29*HK33*HK34 + HK30*HK34*HK40 + HK31*HK34*HK38 + HK32*HK34*HK39 + R_LOS);


// Observation Jacobians
Hfusion.at<0>() = HK3*HK5;
Hfusion.at<1>() = HK5*HK7;
Hfusion.at<2>() = HK5*HK8;
Hfusion.at<3>() = HK5*HK9;
Hfusion.at<4>() = HK22*HK4;
Hfusion.at<5>() = HK27*HK4;
Hfusion.at<6>() = HK28*HK4;
Hfusion.at<7>() = 0;
Hfusion.at<8>() = 0;
Hfusion.at<9>() = 0;
Hfusion.at<10>() = 0;
Hfusion.at<11>() = 0;
Hfusion.at<12>() = 0;
Hfusion.at<13>() = 0;
Hfusion.at<14>() = 0;
Hfusion.at<15>() = 0;
Hfusion.at<16>() = 0;
Hfusion.at<17>() = 0;
Hfusion.at<18>() = 0;
Hfusion.at<19>() = 0;
Hfusion.at<20>() = 0;
Hfusion.at<21>() = 0;
Hfusion.at<22>() = 0;
Hfusion.at<23>() = 0;


// Kalman gains
Kfusion(0) = HK33*HK41;
Kfusion(1) = HK40*HK41;
Kfusion(2) = HK38*HK41;
Kfusion(3) = HK39*HK41;
Kfusion(4) = HK37*HK41;
Kfusion(5) = HK36*HK41;
Kfusion(6) = HK35*HK41;
Kfusion(7) = HK41*(HK22*P(4,7) + HK27*P(5,7) + HK28*P(6,7) + HK29*P(0,7) + HK30*P(1,7) + HK31*P(2,7) + HK32*P(3,7));
Kfusion(8) = HK41*(HK22*P(4,8) + HK27*P(5,8) + HK28*P(6,8) + HK29*P(0,8) + HK30*P(1,8) + HK31*P(2,8) + HK32*P(3,8));
Kfusion(9) = HK41*(HK22*P(4,9) + HK27*P(5,9) + HK28*P(6,9) + HK29*P(0,9) + HK30*P(1,9) + HK31*P(2,9) + HK32*P(3,9));
Kfusion(10) = HK41*(HK22*P(4,10) + HK27*P(5,10) + HK28*P(6,10) + HK29*P(0,10) + HK30*P(1,10) + HK31*P(2,10) + HK32*P(3,10));
Kfusion(11) = HK41*(HK22*P(4,11) + HK27*P(5,11) + HK28*P(6,11) + HK29*P(0,11) + HK30*P(1,11) + HK31*P(2,11) + HK32*P(3,11));
Kfusion(12) = HK41*(HK22*P(4,12) + HK27*P(5,12) + HK28*P(6,12) + HK29*P(0,12) + HK30*P(1,12) + HK31*P(2,12) + HK32*P(3,12));
Kfusion(13) = HK41*(HK22*P(4,13) + HK27*P(5,13) + HK28*P(6,13) + HK29*P(0,13) + HK30*P(1,13) + HK31*P(2,13) + HK32*P(3,13));
Kfusion(14) = HK41*(HK22*P(4,14) + HK27*P(5,14) + HK28*P(6,14) + HK29*P(0,14) + HK30*P(1,14) + HK31*P(2,14) + HK32*P(3,14));
Kfusion(15) = HK41*(HK22*P(4,15) + HK27*P(5,15) + HK28*P(6,15) + HK29*P(0,15) + HK30*P(1,15) + HK31*P(2,15) + HK32*P(3,15));
Kfusion(16) = HK41*(HK22*P(4,16) + HK27*P(5,16) + HK28*P(6,16) + HK29*P(0,16) + HK30*P(1,16) + HK31*P(2,16) + HK32*P(3,16));
Kfusion(17) = HK41*(HK22*P(4,17) + HK27*P(5,17) + HK28*P(6,17) + HK29*P(0,17) + HK30*P(1,17) + HK31*P(2,17) + HK32*P(3,17));
Kfusion(18) = HK41*(HK22*P(4,18) + HK27*P(5,18) + HK28*P(6,18) + HK29*P(0,18) + HK30*P(1,18) + HK31*P(2,18) + HK32*P(3,18));
Kfusion(19) = HK41*(HK22*P(4,19) + HK27*P(5,19) + HK28*P(6,19) + HK29*P(0,19) + HK30*P(1,19) + HK31*P(2,19) + HK32*P(3,19));
Kfusion(20) = HK41*(HK22*P(4,20) + HK27*P(5,20) + HK28*P(6,20) + HK29*P(0,20) + HK30*P(1,20) + HK31*P(2,20) + HK32*P(3,20));
Kfusion(21) = HK41*(HK22*P(4,21) + HK27*P(5,21) + HK28*P(6,21) + HK29*P(0,21) + HK30*P(1,21) + HK31*P(2,21) + HK32*P(3,21));
Kfusion(22) = HK41*(HK22*P(4,22) + HK27*P(5,22) + HK28*P(6,22) + HK29*P(0,22) + HK30*P(1,22) + HK31*P(2,22) + HK32*P(3,22));
Kfusion(23) = HK41*(HK22*P(4,23) + HK27*P(5,23) + HK28*P(6,23) + HK29*P(0,23) + HK30*P(1,23) + HK31*P(2,23) + HK32*P(3,23));


// Y Axis Equations
// Sub Expressions
const float HK0 = Tbs(0,1)*q1;
const float HK1 = Tbs(0,2)*q0;
const float HK2 = Tbs(0,0)*q2;
const float HK3 = HK0 + HK1 - HK2;
const float HK4 = Tbs(0,0)*q3;
const float HK5 = Tbs(0,1)*q0;
const float HK6 = Tbs(0,2)*q1;
const float HK7 = HK4 + HK5 - HK6;
const float HK8 = Tbs(0,0)*q0;
const float HK9 = Tbs(0,2)*q2;
const float HK10 = Tbs(0,1)*q3;
const float HK11 = -HK10 + HK8 + HK9;
const float HK12 = HK11*vn + HK3*vd + HK7*ve;
const float HK13 = 1.0F/range;
const float HK14 = 2*HK13;
const float HK15 = Tbs(0,0)*q1 + Tbs(0,1)*q2 + Tbs(0,2)*q3;
const float HK16 = HK15*vn + HK7*vd;
const float HK17 = HK16 - HK3*ve;
const float HK18 = HK15*ve + HK3*vn;
const float HK19 = -HK11*vd + HK18;
const float HK20 = HK11*ve + HK15*vd;
const float HK21 = HK20 - HK7*vn;
const float HK22 = q0*q3;
const float HK23 = q1*q2;
const float HK24 = 2*Tbs(0,1);
const float HK25 = powf(q1, 2);
const float HK26 = powf(q2, 2);
const float HK27 = -HK26;
const float HK28 = powf(q0, 2);
const float HK29 = powf(q3, 2);
const float HK30 = HK28 - HK29;
const float HK31 = q0*q2;
const float HK32 = q1*q3;
const float HK33 = 2*Tbs(0,2);
const float HK34 = HK33*(HK31 + HK32) + Tbs(0,0)*(HK25 + HK27 + HK30);
const float HK35 = -HK24*(HK22 - HK23) + HK34;
const float HK36 = q0*q1;
const float HK37 = q2*q3;
const float HK38 = -HK25;
const float HK39 = 2*Tbs(0,0);
const float HK40 = HK39*(HK22 + HK23) + Tbs(0,1)*(HK26 + HK30 + HK38);
const float HK41 = -HK33*(HK36 - HK37) + HK40;
const float HK42 = HK24*(HK36 + HK37) + Tbs(0,2)*(HK27 + HK28 + HK29 + HK38);
const float HK43 = -HK39*(HK31 - HK32) + HK42;
const float HK44 = 2*HK12;
const float HK45 = 2*HK16 + 2*ve*(-HK0 - HK1 + HK2);
const float HK46 = 2*HK18 + 2*vd*(HK10 - HK8 - HK9);
const float HK47 = 2*HK20 + 2*vn*(-HK4 - HK5 + HK6);
const float HK48 = HK24*(-HK22 + HK23) + HK34;
const float HK49 = HK33*(-HK36 + HK37) + HK40;
const float HK50 = HK39*(-HK31 + HK32) + HK42;
const float HK51 = HK44*P(0,0) + HK45*P(0,1) + HK46*P(0,2) + HK47*P(0,3) + HK48*P(0,4) + HK49*P(0,5) + HK50*P(0,6);
const float HK52 = powf(range, -2);
const float HK53 = HK44*P(0,6) + HK45*P(1,6) + HK46*P(2,6) + HK47*P(3,6) + HK48*P(4,6) + HK49*P(5,6) + HK50*P(6,6);
const float HK54 = HK44*P(0,5) + HK45*P(1,5) + HK46*P(2,5) + HK47*P(3,5) + HK48*P(4,5) + HK49*P(5,5) + HK50*P(5,6);
const float HK55 = HK44*P(0,4) + HK45*P(1,4) + HK46*P(2,4) + HK47*P(3,4) + HK48*P(4,4) + HK49*P(4,5) + HK50*P(4,6);
const float HK56 = HK44*P(0,2) + HK45*P(1,2) + HK46*P(2,2) + HK47*P(2,3) + HK48*P(2,4) + HK49*P(2,5) + HK50*P(2,6);
const float HK57 = 2*HK52;
const float HK58 = HK44*P(0,3) + HK45*P(1,3) + HK46*P(2,3) + HK47*P(3,3) + HK48*P(3,4) + HK49*P(3,5) + HK50*P(3,6);
const float HK59 = HK44*P(0,1) + HK45*P(1,1) + HK46*P(1,2) + HK47*P(1,3) + HK48*P(1,4) + HK49*P(1,5) + HK50*P(1,6);
const float HK60 = HK13/(HK17*HK57*HK59 + HK19*HK56*HK57 + HK21*HK57*HK58 + HK35*HK52*HK55 + HK41*HK52*HK54 + HK43*HK52*HK53 + HK44*HK51*HK52 + R_LOS);


// Observation Jacobians
Hfusion.at<0>() = -HK12*HK14;
Hfusion.at<1>() = -HK14*HK17;
Hfusion.at<2>() = -HK14*HK19;
Hfusion.at<3>() = -HK14*HK21;
Hfusion.at<4>() = -HK13*HK35;
Hfusion.at<5>() = -HK13*HK41;
Hfusion.at<6>() = -HK13*HK43;
Hfusion.at<7>() = 0;
Hfusion.at<8>() = 0;
Hfusion.at<9>() = 0;
Hfusion.at<10>() = 0;
Hfusion.at<11>() = 0;
Hfusion.at<12>() = 0;
Hfusion.at<13>() = 0;
Hfusion.at<14>() = 0;
Hfusion.at<15>() = 0;
Hfusion.at<16>() = 0;
Hfusion.at<17>() = 0;
Hfusion.at<18>() = 0;
Hfusion.at<19>() = 0;
Hfusion.at<20>() = 0;
Hfusion.at<21>() = 0;
Hfusion.at<22>() = 0;
Hfusion.at<23>() = 0;


// Kalman gains
Kfusion(0) = -HK51*HK60;
Kfusion(1) = -HK59*HK60;
Kfusion(2) = -HK56*HK60;
Kfusion(3) = -HK58*HK60;
Kfusion(4) = -HK55*HK60;
Kfusion(5) = -HK54*HK60;
Kfusion(6) = -HK53*HK60;
Kfusion(7) = -HK60*(HK44*P(0,7) + HK45*P(1,7) + HK46*P(2,7) + HK47*P(3,7) + HK48*P(4,7) + HK49*P(5,7) + HK50*P(6,7));
Kfusion(8) = -HK60*(HK44*P(0,8) + HK45*P(1,8) + HK46*P(2,8) + HK47*P(3,8) + HK48*P(4,8) + HK49*P(5,8) + HK50*P(6,8));
Kfusion(9) = -HK60*(HK44*P(0,9) + HK45*P(1,9) + HK46*P(2,9) + HK47*P(3,9) + HK48*P(4,9) + HK49*P(5,9) + HK50*P(6,9));
Kfusion(10) = -HK60*(HK44*P(0,10) + HK45*P(1,10) + HK46*P(2,10) + HK47*P(3,10) + HK48*P(4,10) + HK49*P(5,10) + HK50*P(6,10));
Kfusion(11) = -HK60*(HK44*P(0,11) + HK45*P(1,11) + HK46*P(2,11) + HK47*P(3,11) + HK48*P(4,11) + HK49*P(5,11) + HK50*P(6,11));
Kfusion(12) = -HK60*(HK44*P(0,12) + HK45*P(1,12) + HK46*P(2,12) + HK47*P(3,12) + HK48*P(4,12) + HK49*P(5,12) + HK50*P(6,12));
Kfusion(13) = -HK60*(HK44*P(0,13) + HK45*P(1,13) + HK46*P(2,13) + HK47*P(3,13) + HK48*P(4,13) + HK49*P(5,13) + HK50*P(6,13));
Kfusion(14) = -HK60*(HK44*P(0,14) + HK45*P(1,14) + HK46*P(2,14) + HK47*P(3,14) + HK48*P(4,14) + HK49*P(5,14) + HK50*P(6,14));
Kfusion(15) = -HK60*(HK44*P(0,15) + HK45*P(1,15) + HK46*P(2,15) + HK47*P(3,15) + HK48*P(4,15) + HK49*P(5,15) + HK50*P(6,15));
Kfusion(16) = -HK60*(HK44*P(0,16) + HK45*P(1,16) + HK46*P(2,16) + HK47*P(3,16) + HK48*P(4,16) + HK49*P(5,16) + HK50*P(6,16));
Kfusion(17) = -HK60*(HK44*P(0,17) + HK45*P(1,17) + HK46*P(2,17) + HK47*P(3,17) + HK48*P(4,17) + HK49*P(5,17) + HK50*P(6,17));
Kfusion(18) = -HK60*(HK44*P(0,18) + HK45*P(1,18) + HK46*P(2,18) + HK47*P(3,18) + HK48*P(4,18) + HK49*P(5,18) + HK50*P(6,18));
Kfusion(19) = -HK60*(HK44*P(0,19) + HK45*P(1,19) + HK46*P(2,19) + HK47*P(3,19) + HK48*P(4,19) + HK49*P(5,19) + HK50*P(6,19));
Kfusion(20) = -HK60*(HK44*P(0,20) + HK45*P(1,20) + HK46*P(2,20) + HK47*P(3,20) + HK48*P(4,20) + HK49*P(5,20) + HK50*P(6,20));
Kfusion(21) = -HK60*(HK44*P(0,21) + HK45*P(1,21) + HK46*P(2,21) + HK47*P(3,21) + HK48*P(4,21) + HK49*P(5,21) + HK50*P(6,21));
Kfusion(22) = -HK60*(HK44*P(0,22) + HK45*P(1,22) + HK46*P(2,22) + HK47*P(3,22) + HK48*P(4,22) + HK49*P(5,22) + HK50*P(6,22));
Kfusion(23) = -HK60*(HK44*P(0,23) + HK45*P(1,23) + HK46*P(2,23) + HK47*P(3,23) + HK48*P(4,23) + HK49*P(5,23) + HK50*P(6,23));


