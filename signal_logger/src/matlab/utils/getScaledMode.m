function [LF_m_scaled RF_m_scaled LH_m_scaled RH_m_scaled] = getScaledMode(data, oldModes, newModes)
global idx_LF_HAA_m idx_LF_HFE_m idx_LF_KFE_m
global idx_RF_HAA_m idx_RF_HFE_m idx_RF_KFE_m
global idx_LH_HAA_m idx_LH_HFE_m idx_LH_KFE_m
global idx_RH_HAA_m idx_RH_HFE_m idx_RH_KFE_m


LF_m_scaled = -ones(length(data),1);
RF_m_scaled = -ones(length(data),1);
LH_m_scaled = -ones(length(data),1);
RH_m_scaled = -ones(length(data),1);


for i=1:length(oldModes)
    LF_m_scaled(data(:,idx_LF_HAA_m)==oldModes(i))=newModes(i);
    RF_m_scaled(data(:,idx_RF_HAA_m)==oldModes(i))=newModes(i);
    LH_m_scaled(data(:,idx_LH_HAA_m)==oldModes(i))=newModes(i);
    RH_m_scaled(data(:,idx_RH_HAA_m)==oldModes(i))=newModes(i);
end