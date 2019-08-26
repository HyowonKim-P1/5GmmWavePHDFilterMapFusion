ti = 18;
for i = 1:para.N_vehicle
    up_UE(i) = Stack(ti).up_UE(i);
end

for i = 1:para.N_vehicle
    up_UE(i) = Resampling(up_UE(i), Stack(ti).V_Est(:,i), state(:,ti,i), para);
end

for i = 1:para.N_vehicle
    up_Map(i) = Stack(ti).up_Map(i);
end