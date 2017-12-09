function [Yaw, Pitch, Roll] = R2YPR(R)

r = R;

Yaw = atan2(r(2, 1), r(1, 1));

Pitch = atan2(r(3, 1), sqrt(r(3, 2) * 2 + r(3, 3) * 2));

Roll = atan2(r(3, 2), r(3, 3));

end