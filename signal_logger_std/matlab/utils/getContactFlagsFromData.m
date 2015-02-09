function [contactFlags] = getContactFlagsFromData(i, data)

global idx_LF_CONTACT_FLAG idx_RF_CONTACT_FLAG idx_LH_CONTACT_FLAG idx_RH_CONTACT_FLAG
contactFlags = [data(i,idx_LF_CONTACT_FLAG) data(i,idx_RF_CONTACT_FLAG) data(i,idx_LH_CONTACT_FLAG) data(i,idx_RH_CONTACT_FLAG)];