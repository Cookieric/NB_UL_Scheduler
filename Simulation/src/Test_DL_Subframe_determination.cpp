#include "rrc_NB.h"
#include <iostream>


uint32_t CSS_NPDCCH_period[3]={0,0,0};
uint32_t USS_NPDCCH_period[3]={0,0,0};

void Initial_UE_Specific_Config(RRCCoonectionSetup_NB * Msg4_S,SIB2_NB * SIB2_NB_S)
{
	//CE 0
	Msg4_S->UE_specificConfig.npdcch_NumRepetitions[0]=SIB2_NB_S->npdcch_NumRepetitions_RA[0];
	Msg4_S->UE_specificConfig.npdcch_StartSF_USS[0]=SIB2_NB_S->npdcch_StartSF_CSS_RA[0];
	Msg4_S->UE_specificConfig.npdcch_Offset_USS[0]=SIB2_NB_S->npdcch_Offset_RA[0];
	//CE 1
	Msg4_S->UE_specificConfig.npdcch_NumRepetitions[1]=SIB2_NB_S->npdcch_NumRepetitions_RA[1];;
	Msg4_S->UE_specificConfig.npdcch_StartSF_USS[1]=SIB2_NB_S->npdcch_StartSF_CSS_RA[1];
	Msg4_S->UE_specificConfig.npdcch_Offset_USS[1]=SIB2_NB_S->npdcch_Offset_RA[1];
	//CE 2
	Msg4_S->UE_specificConfig.npdcch_NumRepetitions[2]=SIB2_NB_S->npdcch_NumRepetitions_RA[2];;
	Msg4_S->UE_specificConfig.npdcch_StartSF_USS[2]=SIB2_NB_S->npdcch_StartSF_CSS_RA[2];
	Msg4_S->UE_specificConfig.npdcch_Offset_USS[2]=SIB2_NB_S->npdcch_Offset_RA[2];
	//BSR
	Msg4_S->bsrConfig.PeriodicBSR_Timer_NB=2;// 2pp is smallest value, 8 pp is default value in 36.331 spec.
	Msg4_S->bsrConfig.RetxBSR_Timer_NB=-1;//disable the timer in NB-IoT
}

int NB_eNB_Init_RRC(MIB_NB * MIB_NB_S, SIB1_NB * SIB1_NB_S, SIB2_NB * SIB2_NB_S,RRCCoonectionSetup_NB * Msg4_S)
{
	//Initial for MIB-NB
	MIB_NB_S->operationModeInfo=2;//guard band deployment
	MIB_NB_S->schedulingInfoSIB1=9;
	LOG("OperationMode:%s\n",NameOperationMode[MIB_NB_S->operationModeInfo]);
	//Initial for SIB1-NB
	SIB1_NB_S->si_WindowLength=160;
	SIB1_NB_S->si_RadioFrameOffset=1;// set to 1 if cell id is even, set to 0 if cell id is odd.
	SIB1_NB_S->si_Periodicity=640;//RF64
	SIB1_NB_S->si_RepetitionPattern=0x01;//0x01 means every2ndRF if (frame&0x01==0)
	SIB1_NB_S->si_TB=208;//208 bits(8 subfrmaes) for SIB2/3,sib2-r13=163 bits,sib3-r13=37 bits,Total: 200 bits
	//Initial for SIB2-NB
	if(Filter_SIB2_NB_Config(SIB2_NB_S)==1);	LOG("\tFilter and Set NPRACH Resource done!\n");
	if(checkFrequencyDomain(SIB2_NB_S)==0)	LOG("\tNPRACH Config CE(0,1,2) Pass FrequencyDomain limitation\n");
	//Initial for UE-Specific RRC Configuration
	Initial_UE_Specific_Config(Msg4_S,SIB2_NB_S);
	//All pp are the same for three CE levels
	// NPDCCH_period=SIB2_NB_S->npdcch_NumRepetitions_RA[0] * SIB2_NB_S->npdcch_StartSF_CSS_RA[0];
	CSS_NPDCCH_period[0]=SIB2_NB_S->npdcch_NumRepetitions_RA[0] * SIB2_NB_S->npdcch_StartSF_CSS_RA[0];
	CSS_NPDCCH_period[1]=SIB2_NB_S->npdcch_NumRepetitions_RA[1] * SIB2_NB_S->npdcch_StartSF_CSS_RA[1];
	CSS_NPDCCH_period[2]=SIB2_NB_S->npdcch_NumRepetitions_RA[2] * SIB2_NB_S->npdcch_StartSF_CSS_RA[2];
	USS_NPDCCH_period[0]=Msg4_S->UE_specificConfig.npdcch_NumRepetitions[0] * Msg4_S->UE_specificConfig.npdcch_StartSF_USS[0];
	USS_NPDCCH_period[1]=Msg4_S->UE_specificConfig.npdcch_NumRepetitions[1] * Msg4_S->UE_specificConfig.npdcch_StartSF_USS[1];
	USS_NPDCCH_period[2]=Msg4_S->UE_specificConfig.npdcch_NumRepetitions[2] * Msg4_S->UE_specificConfig.npdcch_StartSF_USS[2];
	return 1;
}

uint32_t check_if_DL_subframe(uint32_t Sche_H_SFN,uint32_t scheFrame,uint32_t scheSubframe,MIB_NB & MIB_NB_S,SIB1_NB & SIB1_NB_S)
{
    // if(scheSubframe==5) DL_Channel_bitmap[i]=NPSS; //Reserve for NPSS, not send MAC_PDU/DCI
    if(scheSubframe==5) return 0; //Reserve for NPSS, not send MAC_PDU/DCI
    else if(((scheFrame & 0x01)==0)&&(scheSubframe==9))
    {
        // LOG("(frame & 0x01):%d\n",(frame & 0x01));
        // DL_Channel_bitmap[i]=NSSS;//Reserve for NSSS, not send MAC_PDU/DCI
        return 0;
    }
    else if(scheSubframe==0)    return 0;//mac_rrc_data_req(get MIB content from RRC)
    // else if(scheSubframe==0)    DL_Channel_bitmap[i]=NPBCH;//mac_rrc_data_req(get MIB content from RRC)

    /*SIB1-NB in subframe #4 of every other frame in 16 continuous frames. Period = 256RF*/
    uint32_t repetitionNum_SIB1=repetiitonSIB1(MIB_NB_S.schedulingInfoSIB1);//4
    uint32_t startRF_SIB1=RFstartSIB1(repetitionNum_SIB1);//0
    uint32_t TBS_Index=MIB_NB_S.schedulingInfoSIB1;
    uint32_t TBS_SIB1=getTBS_SIB1(TBS_Index);
    uint32_t repetitionOffset=sib1_Period/repetitionNum_SIB1;//64
	uint32_t T=Sche_H_SFN * 10240+scheFrame * 10+scheSubframe;
	uint32_t SIB1_T=T%repetitionOffset;//144 mod 64=16
    if((0<=SIB1_T)&&(SIB1_T<=16))   shcedSIB1=true;//0~15
    else	shcedSIB1=false;
    //Bug fixed(frame & 0x01==0)-->((frame & 0x01)==0)
    if (shcedSIB1&&((scheFrame & 0x01)==sib1_startingRF)&&(scheSubframe==4))
    {
        // DL_Channel_bitmap[i]=SIB1;
        return 0;
    }
    uint32_t t_si_Period;*/
    //1,3,5...15,...65,67,...scheFrame
	uint32_t SIB2_T=(T%SIB1_NB_S.si_Periodicity);//131%640=131,132%640=132,...143%640=143...651%640=11....799%640=159
    if((0<=SIB2_T)&&(SIB2_T<SIB1_NB_S.si_WindowLength)&&((scheFrame & 0x01)== SIB1_NB_S.si_RadioFrameOffset))
    {
        // DL_Channel_bitmap[i]=SIB23;
        return 0;
    }
    return T;
}
