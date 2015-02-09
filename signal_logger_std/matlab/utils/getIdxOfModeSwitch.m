function [idx_LF_m, idx_RF_m, idx_LH_m, idx_RH_m] = getIdxOfModeSwitch(data)

global idx_LF_HAA_m idx_LF_HFE_m idx_LF_KFE_m
global idx_RF_HAA_m idx_RF_HFE_m idx_RF_KFE_m
global idx_LH_HAA_m idx_LH_HFE_m idx_LH_KFE_m
global idx_RH_HAA_m idx_RH_HFE_m idx_RH_KFE_m

x = data(2:end,idx_LF_HAA_m);
y = data(1:end-1,idx_LF_HAA_m);
idx_LF_m = find((x(1:end)~=y(1:end)));
x = data(2:end,idx_RF_HAA_m);
y = data(1:end-1,idx_RF_HAA_m);
idx_RF_m = find((x(1:end)~=y(1:end)));
x = data(2:end,idx_LH_HAA_m);
y = data(1:end-1,idx_LH_HAA_m);
idx_LH_m = find((x(1:end)~=y(1:end)));
x = data(2:end,idx_RH_HAA_m);
y = data(1:end-1,idx_RH_HAA_m);
idx_RH_m = find((x(1:end)~=y(1:end)));