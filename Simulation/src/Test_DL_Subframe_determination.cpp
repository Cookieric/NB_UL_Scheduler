#include "rrc_NB.h"
#include "type_NB.h"
#include "dci_NB.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <list>
#include <fstream>
using namespace std;

#define LOG(...) printf(__VA_ARGS__)

typedef struct Sche_RES
{
	list<struct HI_DCI0_request> DCI_L;
	//DL config request
	//Tx request
}Sche_RES_t;

char NameOperationMode[num_operationMode][20] = {
	"inband-SamePCI",
	"inband-DifferentPCI",
	"guardband",
	"standalone"
};

uint32_t Sche_res(frame_t,sub_frame_t,Sche_RES_t &);

uint32_t *DL_Channel_bitmap;
uint32_t Cell_id=0;//Fixed base on NPSS/NSSS Calculation

uint32_t CSS_NPDCCH_period[3]={0,0,0};
uint32_t USS_NPDCCH_period[3]={0,0,0};
vector<uint32_t> Searchspace(8,0);
vector<vector<uint32_t> > locationS(3,Searchspace);
uint32_t DCI_Rep[3];
uint32_t T_DCI_Resource[2]={0};
Sche_RES_t Sche_Response;

uint8_t LOG_Flag=0;

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

int Filter_SIB2_NB_Config(SIB2_NB * SIB2_NB_S)
{
	int rachperiod[8]={40,80,160,240,320,640,1280,2560};
	int rachstart[8]={8,16,32,64,128,256,512,1024};
	int rachrepeat[8]={1,2,4,8,16,32,64,128};
	int rawindow[8]={2,3,4,5,6,7,8,10};// Not include in MAC_Command in shell script
	int rmax[12]={1,2,4,8,16,32,64,128,256,512,1024,2048};
	float gvalue[8]={1.5,2,4,8,16,32,48,64};
	int candidate[4]={1,2,4,8};


	float pdcchoffset[4]={0,0.125,0.25,0.375};


	int dlcqi[13]={0,1,2,3,4,5,6,7,8,9,10,11,12};
	int dlrepeat[16]={1,2,4,8,16,32,64,128,192,256,384,512,768,1024,1536,2048};
	int rachscofst[7]={0,12,24,36,2,18,34};
	int rachnumsc[4]={12,24,36,48};

	//limitCondition1
	int L_rachperiod=sizeof(rachperiod)/sizeof(int);
	int L_rachrepeat=sizeof(rachrepeat)/sizeof(int);
	int L_rachstart=sizeof(rachstart)/sizeof(int);

	//limitCondition2
	int L_candidate=sizeof(candidate)/sizeof(int);
	int L_rmax=sizeof(rmax)/sizeof(int);
	// LOG("L_candidate:%d,L_rmax:%d",L_candidate,L_rmax);
	//Not limitCondition
	int L_rawindow=sizeof(rawindow)/sizeof(int);
	int L_dlcqi=sizeof(dlcqi)/sizeof(int);
	int L_dlrepeat=sizeof(dlrepeat)/sizeof(int);
	float L_gvalue=sizeof(gvalue)/sizeof(float);

	int limitCondition1[500][3];
	int cnt=0;
	int i,j,k;
	/*limitation condition*/
	for (i = 0; i < L_rachperiod; ++i)
	{
		for (j = 0; j < L_rachrepeat; ++j)
			for (k = 0; k < L_rachstart; ++k)
				if(rachperiod[i]>1.4 * 4 * rachrepeat[j]&&rachperiod[i]>rachstart[k])
				{
					limitCondition1[cnt][0]=rachperiod[i];
					limitCondition1[cnt][1]=rachrepeat[j];
					limitCondition1[cnt][2]=rachstart[k];
					// LOG("%d,%d,%d\n",limitCondition1[cnt][0],limitCondition1[cnt][1],limitCondition1[cnt][2]);
					// if(limitCondition1[cnt][0]==80&&limitCondition1[cnt][1]==2&&limitCondition1[cnt][2]==8)
					// 	LOG("Test for limitation\n");
					++cnt;
				}
	}
	// printf("%d\n", cnt);
	printf("Filter Running:\n");
	int setdone=0;
	// int cnt1=0;
	//number_of_target_SNR=3;
	float OffsetSNR[3]={0,10,20};
	int l,m,n;
	for (l= 0; l < cnt; ++l)
	{
		for (m = 0; m < L_candidate; ++m)
			for (n = 0; n < L_rmax; ++n)
				if(candidate[m]<=rmax[n])
					{
						if(setdone==3)
						{
							// SIB2_NB_S->flag_NPRACH_Change=true;
							return 1;
						}
						if(limitCondition1[l][0]==1280&&limitCondition1[l][1]==1&&limitCondition1[l][2]==8&&rmax[n]==8&&candidate[m]==8)
						{
							// LOG("Test CE0\n");
							SIB2_NB_S->target_SNR[0]=14.25-OffsetSNR[0];
							SIB2_NB_S->CE_Level[0]=0;
							SIB2_NB_S->period[0]=limitCondition1[l][0];
							SIB2_NB_S->rep[0]=limitCondition1[l][1];
							SIB2_NB_S->start_time[0]=limitCondition1[l][2];
							SIB2_NB_S->num_Subcarrier[0]=24;
							SIB2_NB_S->subcarrier_Offset[0]=0;
							SIB2_NB_S->npdcch_NumRepetitions_RA[0]=rmax[n];
							SIB2_NB_S->npdcch_StartSF_CSS_RA[0]=16;//G
							SIB2_NB_S->npdcch_Offset_RA[0]=0;//Alpha offset
							SIB2_NB_S->rawindow[0]=2;//2pp
							SIB2_NB_S->candidate[0]=candidate[m];//8, DCI RepNum(00)-->rmax[m]/8=R(1)
							setdone++;
						}
						else if(limitCondition1[l][0]==1280&&limitCondition1[l][1]==2&&limitCondition1[l][2]==8&&rmax[n]==16&&candidate[m]==8)
						{
							// LOG("Test CE1\n");
							SIB2_NB_S->target_SNR[1]=14.25-OffsetSNR[1];
							SIB2_NB_S->CE_Level[1]=1;
							SIB2_NB_S->period[1]=limitCondition1[l][0];
							SIB2_NB_S->rep[1]=limitCondition1[l][1];
							SIB2_NB_S->start_time[1]=limitCondition1[l][2];
							SIB2_NB_S->num_Subcarrier[1]=12;
							SIB2_NB_S->subcarrier_Offset[1]=24;
							SIB2_NB_S->npdcch_NumRepetitions_RA[1]=rmax[n];
							SIB2_NB_S->npdcch_StartSF_CSS_RA[1]=16;//G
							SIB2_NB_S->npdcch_Offset_RA[1]=0;
							SIB2_NB_S->rawindow[1]=2;
							SIB2_NB_S->candidate[1]=candidate[m];//8, DCI RepNum(00)-->rmax[m]/8=R(2)
							setdone++;
						}
						else if(limitCondition1[l][0]==1280&&limitCondition1[l][1]==4&&limitCondition1[l][2]==8&&rmax[n]==32&&candidate[m]==8)
						{
							// LOG("Test CE2\n");
							SIB2_NB_S->target_SNR[2]=14.25-OffsetSNR[2];
							SIB2_NB_S->CE_Level[2]=2;
							SIB2_NB_S->period[2]=limitCondition1[l][0];
							SIB2_NB_S->rep[2]=limitCondition1[l][1];
							SIB2_NB_S->start_time[2]=limitCondition1[l][2];
							SIB2_NB_S->num_Subcarrier[2]=12;
							SIB2_NB_S->subcarrier_Offset[2]=36;
							SIB2_NB_S->npdcch_NumRepetitions_RA[2]=rmax[n];
							SIB2_NB_S->npdcch_StartSF_CSS_RA[2]=16;//G
							SIB2_NB_S->npdcch_Offset_RA[2]=0;
							SIB2_NB_S->rawindow[2]=2;
							SIB2_NB_S->candidate[2]=candidate[m];//8, DCI RepNum(00)-->rmax[m]/8=R(4)
							setdone++;
						}
						// ++cnt1;
					}
	}
	LOG("Program should Not execute here\n");
	return 0;
}

int checkFrequencyDomain(SIB2_NB * SIB2_NB_S)
{
	int checkFreqArray[48]={0};
	int errorReason=-1;
	int i,j;
	for (i = 0; i < 3; ++i)
	{
		if(i==0)
		{
			for (j = SIB2_NB_S->subcarrier_Offset[i]; j < SIB2_NB_S->subcarrier_Offset[i]+SIB2_NB_S->num_Subcarrier[i]; ++j)
			{
				if(checkFreqArray[j]==0)	checkFreqArray[j]=1;
				else	errorReason=0;
			}
		}
		else if(i==1)
		{
			for (j = SIB2_NB_S->subcarrier_Offset[i]; j < SIB2_NB_S->subcarrier_Offset[i]+SIB2_NB_S->num_Subcarrier[i]; ++j)
			{
				if(checkFreqArray[j]==0)	checkFreqArray[j]=2;
				else	errorReason=1;
			}
		}
		else if(i==2)
		{
			for (j = SIB2_NB_S->subcarrier_Offset[i]; j < SIB2_NB_S->subcarrier_Offset[i]+SIB2_NB_S->num_Subcarrier[i]; ++j)
			{
				if(checkFreqArray[j]==0)	checkFreqArray[j]=3;
				else	errorReason=2;
			}
		}
		if(errorReason!=-1)
		{
			LOG("Error: the NPRACH-Config(CE%d) exceed FreqDomain resource!!\n",errorReason);
			system("pause");
			exit(1);
		}
	}
	return 0;
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

uint32_t repetiitonSIB1(uint32_t schedulingInfoSIB1)
{
	uint32_t repetition[3]={4,8,16};
	int i;
	for (i = 0; i < 11; ++i)
	{
		if(schedulingInfoSIB1==i)	return repetition[i%3];
	}
}

uint32_t RFstartSIB1(uint32_t repetitionNum_SIB1)
{
	if(repetitionNum_SIB1==4)
	{
		if(Cell_id%4==0)	return 0;
		else if(Cell_id%4==1)	return 12;
		else if(Cell_id%4==2)	return 32;
		else if(Cell_id%4==3)	return 48;
	}
	else if(repetitionNum_SIB1==8)
	{
		if(Cell_id%2==0)	return 0;
		else if(Cell_id%2==1)	return 16;
	}
	else if(repetitionNum_SIB1==16)
	{
		if(Cell_id%2==0)	return 0;
		else if(Cell_id%2==1)	return 1;
	}
	else
	{
		LOG("Abnormal repetitionNum_SIB1!\n");
		exit(1);
	}
}

bool compareMyType4 (const HI_DCI0_request_t &a, const HI_DCI0_request_t &b)
{
	return a.DCI_Format.DCI_UL_PDU.endTime > b.DCI_Format.DCI_UL_PDU.endTime;
}

bool compareMyType5 (const HI_DCI0_request_t &a, const HI_DCI0_request_t &b)
{
	return a.DCI_Format.DCI_UL_PDU.startTime < b.DCI_Format.DCI_UL_PDU.startTime;
}

const uint32_t TBS_SIB1[16]={208,208,208,328,328,328,440,440,440,680,680,680,0,0,0};//0:Reserved
uint32_t getTBS_SIB1(uint32_t TBS_Index)
{
	return TBS_SIB1[TBS_Index];
}

#define sib1_startingRF 0 // it means SIB1 start sched at first RF.  1 means start at second RF.
uint32_t sib1_Period=256;//256 RF
bool shcedSIB1=false;

uint32_t fill_DL_subframe_bitmap(uint32_t Sche_H_SFN,uint32_t scheFrame,uint32_t scheSubframe,MIB_NB & MIB_NB_S,SIB1_NB & SIB1_NB_S,uint32_t *DL_Channel_bitmap,uint32_t T_bitmap)
{
   if(scheSubframe==5) DL_Channel_bitmap[T_bitmap]=NPSS; //Reserve for NPSS, not send MAC_PDU/DCI
    // if(scheSubframe==5) return 0; //Reserve for NPSS, not send MAC_PDU/DCI
    else if(((scheFrame & 0x01)==0)&&(scheSubframe==9))
    {
        // LOG("(frame & 0x01):%d\n",(frame & 0x01));
        DL_Channel_bitmap[T_bitmap]=NSSS;//Reserve for NSSS, not send MAC_PDU/DCI
        // return 0;
    }
    // else if(scheSubframe==0)    return 0;//mac_rrc_data_req(get MIB content from RRC)
    else if(scheSubframe==0)    DL_Channel_bitmap[T_bitmap]=NPBCH;//mac_rrc_data_req(get MIB content from RRC)

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
        DL_Channel_bitmap[T_bitmap]=SIB1;
        // return 0;
    }
    uint32_t t_si_Period;
    //1,3,5...15,...65,67,...scheFrame
	uint32_t SIB2_T=(T%SIB1_NB_S.si_Periodicity);//131%640=131,132%640=132,...143%640=143...651%640=11....799%640=159
    if((0<=SIB2_T)&&(SIB2_T<SIB1_NB_S.si_WindowLength)&&((scheFrame & 0x01)== SIB1_NB_S.si_RadioFrameOffset))
    {
        DL_Channel_bitmap[T_bitmap]=SIB23;
        // return 0;
    }
    if(DL_Channel_bitmap[T_bitmap]==NA)	return T;
    else	return 0;
}


uint32_t check_if_DL_subframe(uint32_t Sche_H_SFN,uint32_t scheFrame,uint32_t scheSubframe,MIB_NB & MIB_NB_S,SIB1_NB & SIB1_NB_S,uint32_t *DL_Channel_bitmap,uint32_t T_bitmap)
{
	uint32_t T=Sche_H_SFN * 10240+scheFrame * 10+scheSubframe;

	/*With DL bitmap*/
	// if(DL_Channel_bitmap[T_bitmap]==NA)	return T;
	// else return 0;

	/*Without DL bitmap*/
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

	uint32_t SIB1_T=T%repetitionOffset;//144 mod 64=16
    if((0<=SIB1_T)&&(SIB1_T<=16))   shcedSIB1=true;//0~15
    else	shcedSIB1=false;
    //Bug fixed(frame & 0x01==0)-->((frame & 0x01)==0)
    if (shcedSIB1&&((scheFrame & 0x01)==sib1_startingRF)&&(scheSubframe==4))
    {
        // DL_Channel_bitmap[i]=SIB1;
        return 0;
    }
    uint32_t t_si_Period;
    //1,3,5...15,...65,67,...scheFrame
	uint32_t SIB2_T=(T%SIB1_NB_S.si_Periodicity);//131%640=131,132%640=132,...143%640=143...651%640=11....799%640=159
    if((0<=SIB2_T)&&(SIB2_T<SIB1_NB_S.si_WindowLength)&&((scheFrame & 0x01)== SIB1_NB_S.si_RadioFrameOffset))
    {
        // DL_Channel_bitmap[i]=SIB23;
        return 0;
    }
    return T;
}

// void NB_schedule_ulsch(uint32_t schedTime,uint32_t CE_Level,uint32_t *DL_Channel_bitmap,SIB2_NB & SIB2_NB_S,RRCCoonectionSetup_NB & Msg4_S)
void NB_schedule_ulsch(uint32_t schedTime,uint32_t CE_Level,MIB_NB & MIB_NB_S,SIB1_NB & SIB1_NB_S,SIB2_NB & SIB2_NB_S,RRCCoonectionSetup_NB & Msg4_S)
{
    uint32_t NPDCCH_period=0,npdcch_Offset=0,T_SearchSpace=0;
	NPDCCH_period=USS_NPDCCH_period[CE_Level];
	npdcch_Offset=Msg4_S.UE_specificConfig.npdcch_Offset_USS[CE_Level];
	T_SearchSpace=Msg4_S.UE_specificConfig.npdcch_NumRepetitions[CE_Level];
	uint32_t scheH_SFN=0;
	frame_t scheFrame=0;
	sub_frame_t scheSubframe=0;
	// uint32_t schedTime=scheH_SFN * 10240+scheFrame * 10+scheSubframe;
	uint32_t offsetSearchSpace=npdcch_Offset * NPDCCH_period;
    scheSubframe=(schedTime+offsetSearchSpace)%10;
    scheFrame=((schedTime+offsetSearchSpace)/10)%1024;
    scheH_SFN=(schedTime+scheSubframe+offsetSearchSpace)/10240;

	uint32_t cntSearchspace=0;//cnt used searchspace and record index of current occupued search space....
    uint32_t cnt_default=0;// cnt # of non-DL subframe

	//Step 1: DCI resource determination(NCCE resource allocation)
	HI_DCI0_request_t DCI_Info={0};
    //Get Aggregation level base on Rmax/R and CE level
    DCI_Info.DCI_Format.DCI_UL_PDU.Aggregation_L=2;
    bool schedStatus=false;
	bool Find_S=true;//find start time of DCI only once
    uint32_t DCI_S=0;
    list<HI_DCI0_request_t> & DCI_List=Sche_Response.DCI_L;

	DL_Channel_bitmap=(uint32_t *)calloc(NPDCCH_period,sizeof(uint32_t));
	uint32_t T_bitmap=0;

	/*With DL bitmap*/
	// while(cntSearchspace<=T_SearchSpace/2)
	// {
	// 	DCI_S=fill_DL_subframe_bitmap(scheH_SFN,scheFrame,scheSubframe,MIB_NB_S,SIB1_NB_S,DL_Channel_bitmap,T_bitmap);
	// 	if(DCI_S!=0)//Get current time of DL subframe
	// 	{
	// 		++cntSearchspace;
	// 	}
	// 	++scheSubframe;
	// 	++T_bitmap;
	// 	if(scheSubframe==10)
	// 	{
	// 		scheSubframe=0;
	// 		++scheFrame;
	// 	}
	// 	if(scheFrame==1024)
	// 	{
	// 		scheFrame=0;
	// 		scheH_SFN++;
 //  		}
	// }
	// cntSearchspace=0;
	// DCI_S=0;
	// T_bitmap=0;
 //    scheSubframe=(schedTime+offsetSearchSpace)%10;
 //    scheFrame=((schedTime+offsetSearchSpace)/10)%1024;
 //    scheH_SFN=(schedTime+scheSubframe+offsetSearchSpace)/10240;

	for (int i=0; i<4;++i)//Test 4 UL DCI for PP of each CE level
	{
    	//cntSearchspace:Index of search sapce,T_SearchSpace:The length of search sapce
		while(cntSearchspace<=T_SearchSpace/2)
		{
			// fout_LOG<<"[Step1]"<<"UE_Id:"<<(*it1).UE_id<<endl;
	        // LOG("scheH_SFN:%d,scheFrame:%d,scheSubframe:%d\n",scheH_SFN,scheFrame,scheSubframe);
			DCI_S=check_if_DL_subframe(scheH_SFN,scheFrame,scheSubframe,MIB_NB_S,SIB1_NB_S,DL_Channel_bitmap,T_bitmap);
	        // LOG("CurrentTime:%d,cntSearchspace:%d\n",DCI_S,cntSearchspace);
			if(DCI_S!=0)//Get current time of DL subframe
			{
				if(DCI_List.empty())
				{
					if(Find_S)
					{
						//j is the maximun DCI size
						for (int j=0;j<locationS[CE_Level].size(); j++)//locationS: 3X8 array
						{
	                        //If Rmax/R is the same for CSS/USS in the same CE level, locationS keep the same
	                        //If Rmax/R is different for CSS/USS in the same CE level, locationS will changed...
	                        //If a NB-IoT UE has a NPUSCH transmission ending in subframe n , the UE is not required to monitor NPDCCH in any subframe starting from subframe n+1 to subframe n+3

                        	//locationS:Possible DCI location in search space
                        	//(0,1,2,3,4,5,6,7)
                        	////(0,2,4,6,8,10,12,14), (0,4,8,12,16,20,24,28)
							if(locationS[CE_Level][j]==cntSearchspace)
							{
                                //Start from NCCE index 0, this could be AL = 1 or 2
                                DCI_Info.DCI_Format.DCI_UL_PDU.NCCE_index=0;
					  			DCI_Info.DCI_Format.DCI_UL_PDU.startTime=DCI_S;
					  			Find_S=false;
							}
						}
					}
					if(!Find_S)   DCI_Info.DCI_Format.DCI_UL_PDU.cntR++;
					if(DCI_Info.DCI_Format.DCI_UL_PDU.cntR==DCI_Rep[CE_Level])
					{
						DCI_Info.DCI_Format.DCI_UL_PDU.endTime=DCI_S;
	                    if(DCI_Info.DCI_Format.DCI_UL_PDU.Aggregation_L==2)
	                    {
	                        T_DCI_Resource[0]=DCI_S;
	                        T_DCI_Resource[1]=DCI_S;
						}
	                    else    T_DCI_Resource[0]=DCI_S;
	                    //num_DCI used by PHY loop generate DCI top
	                    // DCI_Info.num_DCI++;
	                    // num_DCI++;
						DCI_List.push_back  (DCI_Info);
						schedStatus=true;
						// ++cntSearchspace;//This should be update with scheFrame/subframes in sync.
						break;
					}
				}
				else//DCI_List is not empty...
				{
	                uint32_t t_NCCE=0;//temp NCCE Index
	                if(DCI_Info.DCI_Format.DCI_UL_PDU.Aggregation_L==1)//For now Rmax/R setting will not go here
	                {
	                    //Find the earlier time of NCCE0 or NCCE1 and record CCE index
	                    if(T_DCI_Resource[0]>T_DCI_Resource[1])
	                    {
	                        t_NCCE=1;
	                        //Check if available DCI resource >= start search space
	                        if((DCI_S > T_DCI_Resource[1])&&(cntSearchspace==0))
	                        {
	                            //If start search space > Time_NCCE0 set NCCE index=0
	                            if(DCI_S > T_DCI_Resource[0])    t_NCCE=0;
	                            T_DCI_Resource[t_NCCE]=DCI_S;
	                        }
	                    }
	                    else
	                    {
	                        t_NCCE=0;
	                        //Check if available DCI resource >= start search space
	                        if((DCI_S > T_DCI_Resource[0])&&(cntSearchspace==0))
	                        {
	                            // don't need this condition here as I always start from ncce 0
	                            // if(DCI_S > T_DCI_Resource[1])    t_NCCE=1;
	                            T_DCI_Resource[t_NCCE]=DCI_S;
	                        }
	                    }
	                    //CCE allocation for DCI's AL=1
	                    if((DCI_S > T_DCI_Resource[t_NCCE])||((DCI_S==T_DCI_Resource[t_NCCE])&&(cntSearchspace==0)))
	                    {
	                        if(Find_S)
	                        {
	                            for (int j=0;j<locationS[CE_Level].size(); j++)//locationS: 3X8 array
	                            {
	                                if(locationS[CE_Level][j]==cntSearchspace)
	                                {
	                                    DCI_Info.DCI_Format.DCI_UL_PDU.NCCE_index=t_NCCE;
	                                    DCI_Info.DCI_Format.DCI_UL_PDU.startTime=DCI_S;
	                                    Find_S=false;
	                                }
	                            }
	                        }
	                        if(!Find_S)  DCI_Info.DCI_Format.DCI_UL_PDU.cntR++;//Bug fixed: Add if(!Find_S)
	                        if(DCI_Info.DCI_Format.DCI_UL_PDU.cntR==DCI_Rep[CE_Level])
	                        {
	                            T_DCI_Resource[DCI_Info.DCI_Format.DCI_UL_PDU.NCCE_index]=DCI_S;
	                            DCI_Info.DCI_Format.DCI_UL_PDU.endTime=DCI_S;
	                            // DCI_Info.num_DCI++;
	                            // num_DCI++;
	                            DCI_List.push_back  (DCI_Info);
	                            schedStatus=true;
	                            // ++cntSearchspace;//This should be update with scheFrame/subframes in sync.
	                            break;
	                        }
	                    }
	                }
	                else if(DCI_Info.DCI_Format.DCI_UL_PDU.Aggregation_L==2)
	                {
	                    //Find the DCI's biggest endTime in DCI_list
	                    DCI_List.sort(compareMyType4);
	                    // for (list<HI_DCI0_request_t>::iterator DCI_it1=DCI_List.begin(); DCI_it1 != DCI_List.end(); ++DCI_it1)
	                    // {
	                        // LOG("startTime:%d,endTime%d,AL:%d\n",(*DCI_it1).DCI_Format.DCI_UL_PDU.startTime,(*DCI_it1).DCI_Format.DCI_UL_PDU.endTime,(*DCI_it1).DCI_Format.DCI_UL_PDU.Aggregation_L);
	                    // }
	                    // typename list<HI_DCI0_request_t>::iterator DCI_it1 = DCI_List.end();
	                    // --DCI_it1;
	                    typename list<HI_DCI0_request_t>::iterator DCI_it1 = DCI_List.begin();
						if((*DCI_it1).DCI_Format.DCI_UL_PDU.endTime < DCI_S)
						{
							if(Find_S)
							{
								for (int j=0;j<locationS[CE_Level].size(); j++)//locationS: 3X8 array
								{
	                                //(0,2,4,6,8,10,12,14), (0,4,8,12,16,20,24,28)
									if(locationS[CE_Level][j]==cntSearchspace)
									{
	                                    DCI_Info.DCI_Format.DCI_UL_PDU.NCCE_index=t_NCCE;
							  			DCI_Info.DCI_Format.DCI_UL_PDU.startTime=DCI_S;
							  			Find_S=false;
									}
								}
							}
							if(!Find_S)  DCI_Info.DCI_Format.DCI_UL_PDU.cntR++;//Bug fixed: Add if(!Find_S)
							if(DCI_Info.DCI_Format.DCI_UL_PDU.cntR==DCI_Rep[CE_Level])
							{
								DCI_Info.DCI_Format.DCI_UL_PDU.endTime=DCI_S;
	                            T_DCI_Resource[0]=DCI_S;
	                            T_DCI_Resource[1]=DCI_S;
								// DCI_Info.num_DCI++;
	                            // num_DCI++;
								DCI_List.push_back  (DCI_Info);
								schedStatus=true;
								// ++cntSearchspace;//This should be update with scheFrame/subframes in sync.
								break;
							}
						}
	                }
				}
				++cntSearchspace;
	    		// LOG("[NB_schedule_dlsch]: frame:%d,subframes:%d,DL_subframe\n",scheFrame,scheSubframe);
	    		// system("pause");
			}
			else	++cnt_default;//Non-DL subframe
			// local timing for ulsch
			++scheSubframe;
			++T_bitmap;
			if(scheSubframe==10)
			{
				scheSubframe=0;
				++scheFrame;
			}
			if(scheFrame==1024)
			{
				scheFrame=0;
				scheH_SFN++;
	  		}
		    // cout<<"cnt_default:"<<cnt_default<<"cntSearchspace: "<<cntSearchspace<<endl;
		    // system("pause");
		}// end of while

	    //Recover DCIs squence base on earlier start time of DCIs in DCI_List .
	    DCI_List.sort(compareMyType5);
		//Check if DCI have available reosurce in this pp.
		if(schedStatus==false)
	    {
	        LOG("No DCIs available reosurce in this pp...\n");
	        system("pause");
	        continue;
	    }
	   	if(LOG_Flag)
		{
			LOG("DCI_List(1,2....m):\n");
			for (list<HI_DCI0_request_t>::iterator DCI_it1=DCI_List.begin(); DCI_it1 != DCI_List.end(); ++DCI_it1)
			{
				LOG("startTime:%d,endTime%d,AL:%d,NCCE_index:%d,Time_NCCE0:%d,Time_NCCE1:%d\n",(*DCI_it1).DCI_Format.DCI_UL_PDU.startTime,(*DCI_it1).DCI_Format.DCI_UL_PDU.endTime,(*DCI_it1).DCI_Format.DCI_UL_PDU.Aggregation_L,(*DCI_it1).DCI_Format.DCI_UL_PDU.NCCE_index,T_DCI_Resource[0],T_DCI_Resource[1]);
			}
			// system("pause");
		}
	}
	// free(DL_Channel_bitmap);



}

void PossibleSearchspace(SIB2_NB & SIB2_NB_S,vector<vector<uint32_t> > & locationS)
{
	for (int i = 0; i < 3; ++i)
	{
		DCI_Rep[i]=SIB2_NB_S.rep[i];
		uint32_t n=SIB2_NB_S.candidate[i];
		locationS[i].resize(n);//Adjust possible search space location base on DCI candidate
	}

	vector<uint32_t>::iterator V_it;
	uint32_t bi=0;
	for(int i=0;i<locationS.size(); i++)//locationS: 3X8 array
	{
		bi=0;
		cout<<"CE"<<i<<" possible search space location: ";
		for (int j=0;j<locationS[i].size(); j++)
		{
			locationS[i][j]=bi*DCI_Rep[i];
			cout << locationS[i][j] << " ";
			// cout<<"bi:"<<bi<<','<<"DCI_R:"<<DCI_Rep[i]<<endl;
			bi++;
		}
		cout << endl;
	}
  	cout<<endl;
  	system("pause");
}

uint32_t Sche_res(frame_t frame,sub_frame_t subframes,Sche_RES_t & Sche_Response)
{
	list<HI_DCI0_request_t> & DCI_List=Sche_Response.DCI_L;
	// typename list<HI_DCI0_request_t>::iterator DCI_it1 = DCI_List.begin();
	while(!DCI_List.empty())
	{
		DCI_List.pop_front ();
	}
}

int main(int argc, char const *argv[])//design simulation base on different argv/argc form bat...
{
	// while(1)
	// {
	// 	static uint32_t CE=0;
	// 	uint32_t pc=CE;
	// 	LOG("CE:%d\n",CE);
	// 	system("pause");
	// 	++CE;
	// 	if(CE==3)	CE=0;
	// }
	// clock_t t;
	// t = clock();
	MIB_NB	MIB_NB_S= {0};//Directly Initialize
	SIB1_NB	SIB1_NB_S= {0};
	SIB2_NB SIB2_NB_S= {0};
	RRCCoonectionSetup_NB Msg4_S={0};
	if(NB_eNB_Init_RRC(&MIB_NB_S, &SIB1_NB_S, &SIB2_NB_S,&Msg4_S))	LOG("Initialize RRC done\n");

	PossibleSearchspace(SIB2_NB_S,locationS);
	uint32_t simTimes=0;
	uint32_t T=0;
	sub_frame_t subframes=0;
	frame_t frame=0;
	uint32_t H_SFN=0;

	ofstream timeCost;
	timeCost.open("../build/Matlab_Result/timeCost.csv", ios::app);
	if(!timeCost.is_open())
	{
	    cout << "Error: the CSV file is not opened!!" << endl;
	    exit(1);
	}
	// t = clock() - t;
	// LOG("Computing Time:%f seconds\n",((float)t)/CLOCKS_PER_SEC);
	clock_t t,temp_T;
	temp_T = clock();
	// LOG("Computing Time:%f ms\n",((float)t)/CLOCKS_PER_SEC);
	while(1)
	{
		T=H_SFN * 10240+frame * 10+subframes;

		if(((simTimes%1000)==0)&&(T==0))	LOG("simTimes:%d\n",simTimes);

		// if(((simTimes%1000)==0)&&(T==0))
		// {
		// 	t = clock() - t;
		// 	timeCost<<simTimes<<","<<((float)t)/CLOCKS_PER_SEC<<endl;
		// }

		for (int i = 0; i < 3; ++i)
		{
			if(((T+1)%USS_NPDCCH_period[i]==0)||(T==0))
			{
				/*DL bitmap*/
				// NB_schedule_MIB(T+1,i,Msg4_S,DL_Channel_bitmap);
				// NB_schedule_SI(T+1,i,Msg4_S,DL_Channel_bitmap,&MIB_NB_S, &SIB1_NB_S);
	           	if(LOG_Flag)
    			{
					LOG("T:%d,Schedule DCIs in USS of next pp(%d~%d) for UL/DL at previous SF of CE %d pp\n",T,T+1,T+1+USS_NPDCCH_period[i],i);
				}
				NB_schedule_ulsch(T+1,i,MIB_NB_S,SIB1_NB_S,SIB2_NB_S,Msg4_S);
			}
		}
		Sche_res(frame,subframes,Sche_Response);

		if(((simTimes%1000)==0)&&(T==0))
		{
			t = clock()-temp_T;
			timeCost<<simTimes<<","<<((float)t)/CLOCKS_PER_SEC<<endl;
		}

		if(T==10000)
		{
			simTimes++;
			subframes=0;
			frame=0;
			H_SFN=0;
			if(simTimes==10000)	break;
			continue;
		}

		++subframes;
		if(subframes==10)
		{
			subframes=0;
			++frame;
		}
		if(frame==1024)
		{
			frame=0;
			H_SFN++;
		}
		// system("pause");
	}
	free(DL_Channel_bitmap);
	t = clock() - temp_T;
	timeCost<<simTimes<<","<<((float)t)/CLOCKS_PER_SEC<<endl;
	LOG("Computing Time:%f s\n",((float)t)/CLOCKS_PER_SEC);
	timeCost.close();
	system("pause");
	return 0;
}

