# SPDX-License-Identifier: GPL-3.0-only
from functools import lru_cache
import json
import numpy as np
PERM=np.array([1,17,9,25,5,21,13,29,3,19,11,27,7,23,15,31,0,16,8,24,4,20,12,28,2,18,10,26,6,22,14,30])

@lru_cache(maxsize=30)
def convolutional_map(d):
    rows=(d+31)//32
    padded=np.r_[np.full(rows*32-d,-1),np.arange(d)].reshape(rows,32)
    index=padded[:,PERM].T.flatten()
    buffer=np.concatenate([np.where(index<0,-1,index+j*d) for j in range(3)])
    return buffer[buffer>=0]

def rrc_decode(payload):
    from pycrate_asn1dir import RRCLTE
    message=RRCLTE.EUTRA_RRC_Definitions.BCCH_DL_SCH_Message
    message.from_uper(payload)
    # JSON normalizes ASN.1 choices and bit strings without inventing fields.
    decoded=json.loads(message.to_json())
    value=message.get_val()
    outer=value.get('message')
    if outer and outer[0]=='c1' and outer[1][0]=='systemInformation':
        extension=outer[1][1]['criticalExtensions']
        if extension[0]=='systemInformation-r8':
            blocks=extension[1]['sib-TypeAndInfo']
            decoded['decoded_system_information_types']=[name for name,block in blocks]
            for name,block in blocks:
                if name=='sib8':
                    from lte_time_fields import sib8_time
                    decoded['decoded_sib8']=sib8_time(block)
    if outer and outer[0]=='c1' and outer[1][0]=='systemInformationBlockType1':
        block=outer[1][1];access=block['cellAccessRelatedInfo']
        plmns=[];mcc=None
        for item in access['plmn-IdentityList']:
            identity=item['plmn-Identity']
            if 'mcc' in identity:mcc=''.join(map(str,identity['mcc']))
            plmns.append(dict(mcc=mcc,mnc=''.join(map(str,identity['mnc']))))
        decoded['decoded_sib1']=dict(plmns=plmns,tracking_area_code=access['trackingAreaCode'][0],
            eutran_cell_identity=access['cellIdentity'][0],frequency_band=block['freqBandIndicator'],
            scheduling=block['schedulingInfoList'],si_window=block['si-WindowLength'])
        def find_values(obj,key):
            values=[]
            if isinstance(obj,dict):
                if key in obj:values.append(obj[key])
                for item in obj.values():values.extend(find_values(item,key))
            elif isinstance(obj,(tuple,list)):
                for item in obj:values.extend(find_values(item,key))
            return values
        hyper=find_values(block,'hyperSFN-r13')
        if hyper:decoded['decoded_sib1']['hyper_sfn']=hyper[0][0]
        advertised=[s for entry in block['schedulingInfoList'] for s in entry['sib-MappingInfo']]
        decoded['decoded_sib1']['sib16_advertised']=any(s.startswith('sibType16') for s in advertised)
    return decoded
