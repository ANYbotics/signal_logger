function [q] = getQFromData(i, data)
global idx_qX idx_qY idx_qZ 
global idx_qAL idx_qBE idx_qGA 
 global idx_LF_HAA_th idx_LF_HFE_th idx_LF_KFE_th 
 global idx_RF_HAA_th idx_RF_HFE_th idx_RF_KFE_th 
 global idx_LH_HAA_th idx_LH_HFE_th idx_LH_KFE_th 
 global idx_RH_HAA_th idx_RH_HFE_th idx_RH_KFE_th


q =  [data(i,idx_qX) data(i,idx_qY) data(i,idx_qZ) data(i,idx_qAL) data(i,idx_qBE) data(i,idx_qGA) data(i,idx_LF_HAA_th) data(i,idx_LF_HFE_th) data(i,idx_LF_KFE_th) data(i,idx_RF_HAA_th) data(i,idx_RF_HFE_th) data(i,idx_RF_KFE_th) data(i,idx_LH_HAA_th) data(i,idx_LH_HFE_th) data(i,idx_LH_KFE_th) data(i,idx_RH_HAA_th) data(i,idx_RH_HFE_th) data(i,idx_RH_KFE_th)];
