#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#define NDEBUG
#include <assert.h>

#include "queue.h"

//MESI 1, Dragon 2, MESI_adv 3
int protocol=0;

FILE *trace_fp[4];
int cache_size=4096;
int associativity=2;
int block_size=32;
uint32_t set_mask=((~0u)<<5)&((~0u)>>21);
uint32_t tag_mask=(~0u)<<11;
uint32_t no_offset_mask=(~0u)<<5;
int block_offset=5;
int set_index_len=6;
int set_number=64;

//state
enum{
	Invalid=0,
	Shared,
	Modified,
	Exclusive,
	Sc,
	Sm
};
//event
enum{
	Nothing=0,
	PrRd,
	PrWr,
	BusRd,//bus read S' use BusRd_
	BusRdX,
	BusUpd,
	Flush
};

typedef struct
{
	uint32_t tag;
	uint32_t state;
	uint64_t timestamp;
}cache_block;

typedef struct 
{
	uint64_t core_cycle;//change in main loop
	uint64_t compute_cycle;//change when issue calculate
	uint64_t idle_cycle;//change in main loop
	uint32_t ls_ins;//change when issue load/store
	uint32_t miss_count;//change when issue load/store miss
	uint32_t access_private;
	uint32_t access_shared;

	uint32_t is_waiting_bus;
	uint32_t busy_remain_cycle;
}core_res;

typedef struct 
{
	node_t bus_queue;

	uint32_t head_remain_cycle;

	uint32_t traffic_bytes;
	uint32_t number_of_iu;
}bus_res;

typedef struct 
{
	node_t bus_queue;

	int request_type;
	int cpu_id;
	uint32_t address;
	uint32_t estimate_cycle;
	uint32_t traffic_bytes;
}bus_req;

typedef cache_block** cache_set;

cache_set cache_core[4];
bus_res bus_status;
core_res cpu_status[4]={0};
int core_finish[4]={0};

int call_FSM(int cpu_id,uint32_t address,cache_block *block_ptr,int event,uint64_t now_cycle);
int MESI_FSM(int cpu_id,uint32_t address,cache_block *block_ptr,int event,uint64_t now_cycle);
int Dragon_FSM(int cpu_id,uint32_t address,cache_block *block_ptr,int event,uint64_t now_cycle);
int MESI_adv_FSM(int cpu_id,uint32_t address,cache_block *block_ptr,int event,uint64_t now_cycle);

int Bus_main(bus_res *bus_ptr);
int Bus_check_share(uint32_t address,uint8_t cpu[4]);
int Bus_share_signal=0;
void Bus_req_generate(int cpu_id,uint32_t address,int request_type);

int Cache_check_hit(cache_block* set,uint32_t address);//return -1 if miss, return index in set if hit
int Cache_choose_replace(cache_block* set);//only called when miss, return the new index in set

//these FSM also change timestamp alongside the state
//return 0 if access private,return 1 if access shared,return <0 if none
int call_FSM(int cpu_id,uint32_t address,cache_block *block_ptr,int event,uint64_t now_cycle)
{
	switch (protocol){
		case 1:return MESI_FSM(cpu_id,address,block_ptr,event,now_cycle);
		case 2:return Dragon_FSM(cpu_id,address,block_ptr,event,now_cycle);
		case 3:return MESI_adv_FSM(cpu_id,address,block_ptr,event,now_cycle);
		default: assert(protocol);
	}
}
int MESI_FSM(int cpu_id,uint32_t address,cache_block *block_ptr,int event,uint64_t now_cycle)
{
	switch (block_ptr->state)
	{
		case Invalid:
		{
			switch (event)
			{
				case PrWr:{
					//BusRdX
					Bus_req_generate(cpu_id,address,BusRdX);
					block_ptr->state=Modified;
					block_ptr->timestamp=now_cycle;
					return 0;
					break;
				}
				case PrRd:{
					Bus_req_generate(cpu_id,address,BusRd);
					//check share
					if(Bus_share_signal){//BudRd
						block_ptr->state=Shared;
					}
					else{//BusRd_
						block_ptr->state=Exclusive;
					}
					block_ptr->timestamp=now_cycle;
					return Bus_share_signal;

					break;
				}
				default:
					break;
			}
			break;
		}
		case Shared:
		{
			switch (event)
			{
				case PrWr:{
					//BudRdx
					Bus_req_generate(cpu_id,address,BusRdX);
					block_ptr->state=Modified;
					block_ptr->timestamp=now_cycle;
					return 1;
					break;
				}
				case PrRd:{
					block_ptr->timestamp=now_cycle;
					return 1;
					break;
				}
				case BusRd:{
					break;
				}
				case BusRdX:{
					block_ptr->state=Invalid;
					break;
				}
				default:{
					fprintf(stderr,"error event in state Shared!\n");
					exit(1);
				}
			}
			break;
		}
		case Exclusive:
		{
			switch (event)
			{
				case PrWr:{
					block_ptr->timestamp=now_cycle;
					return 0;
					break;
				}
				case PrRd:{
					block_ptr->timestamp=now_cycle;
					return 0;
					break;
				}
				case BusRd:{
					block_ptr->state=Shared;
					break;
				}
				case BusRdX:{
					block_ptr->state=Invalid;
					//flush, but act the same time when readx
					break;
				}
			}
			break;
		}
		case Modified:
		{
			switch (event)
			{
				case PrWr:{
					block_ptr->timestamp=now_cycle;
					return 0;
					break;
				}
				case PrRd:{
					block_ptr->timestamp=now_cycle;
					return 0;
					break;
				}
				case BusRd:{
					block_ptr->state=Shared;
					//flush,but act the same time when read
					break;
				}
				case BusRdX:{
					block_ptr->state=Invalid;
					//flush,but act the same time when readX
					break;
				}
			}
			break;
		}
	}
	return -1;
}
int Dragon_FSM(int cpu_id,uint32_t address,cache_block *block_ptr,int event,uint64_t now_cycle)
{
	switch (block_ptr->state)
	{
		case Invalid://when set is full ,invalid one and excute replace
		{
			switch (event)
			{
				case PrRd:{
					Bus_req_generate(cpu_id,address,BusRd);
					//check share
					if(Bus_share_signal){//BusRd
						block_ptr->state=Sc;
					}
					else{//BusRd_
						block_ptr->state=Exclusive;
					}
					block_ptr->timestamp=now_cycle;
					return Bus_share_signal;
					break;
				}
				case PrWr:{
					Bus_req_generate(cpu_id,address,BusRd);
					//check share
					if(Bus_share_signal){//BusRd
						//then BusUpd
						Bus_req_generate(cpu_id,address,BusUpd);
						block_ptr->state=Sm;
					}
					else{//BusRd_
						block_ptr->state=Modified;
					}
					block_ptr->timestamp=now_cycle;
					return Bus_share_signal;
					break;
				}
				default:{
					fprintf(stderr, "wrong event in Invalid\n");
					exit(1);
				}
			}
			break;
		}
		case Sc:
		{
			switch (event)
			{
				case PrWr:{
					Bus_req_generate(cpu_id,address,BusUpd);
					//check share
					if(Bus_share_signal)//BusUpd(S)
						block_ptr->state=Sm;
					else//BusUpd(S')
						block_ptr->state=Modified;

					block_ptr->timestamp=now_cycle;
					return 1;
					break;
				}
				case PrRd:{
					block_ptr->timestamp=now_cycle;
					return 1;
					break;
				}
				case BusRd:{
					break;
				}
				case BusUpd:{
					//Update, work the same time as busupd
					//update shouldn't change the timestamp
					break;
				}
			}
			break;
		}
		case Sm:
		{
			switch (event)
			{
				case PrWr:{
					Bus_req_generate(cpu_id,address,BusUpd);
					//check share
					if(Bus_share_signal)//BusUpd(S)
						;
					else//BusUpd(S')
						block_ptr->state=Modified;

					block_ptr->timestamp=now_cycle;
					return 1;
					break;
				}
				case PrRd:{
					block_ptr->timestamp=now_cycle;
					return 1;
					break;
				}
				case BusRd:{
					//flush, work the same time when busrd
					break;
				}
				case BusUpd:{
					//update, work the same time when busupd
					block_ptr->state=Sc;
					break;
				}
			}
			break;
		}
		case Modified:
		{
			switch (event)
			{
				case PrWr:{
					block_ptr->timestamp=now_cycle;
					return 0;
					break;
				}
				case PrRd:{
					block_ptr->timestamp=now_cycle;
					return 0;
					break;
				}
				case BusRd:{
					block_ptr->state=Sm;
					//flush, work the same time as busrd
					break;
				}
				case BusUpd:{//error
					fprintf(stderr,"error BusUpd in state Modified!\n");
					exit(1);
					break;
				}
			}
			break;
		}
	}
	return -1;
}
int MESI_adv_FSM(int cpu_id,uint32_t address,cache_block *block_ptr,int event,uint64_t now_cycle)
{
	switch (block_ptr->state)
	{
		case Invalid:
		{
			switch (event)
			{
				case PrWr:{
					break;
				}
				case PrRd:{
					break;
				}
				case BusRd:{
					break;
				}
				case BusRd_:{
					break;
				}
				case BusRdX:{
					break;
				}
			}
			break;
		}
		case Shared:
		{
			switch (event)
			{
				case PrWr:{
					break;
				}
				case PrRd:{
					break;
				}
				case BusRd:{
					break;
				}
				case BusRd_:{
					break;
				}
				case BusRdX:{
					break;
				}
			}
			break;
		}
		case Exclusive:
		{
			switch (event)
			{
				case PrWr:{
					break;
				}
				case PrRd:{
					break;
				}
				case BusRd:{
					break;
				}
				case BusRd_:{
					break;
				}
				case BusRdX:{
					break;
				}
			}
			break;
		}
		case Modified:
		{
			switch (event)
			{
				case PrWr:{
					break;
				}
				case PrRd:{
					break;
				}
				case BusRd:{
					break;
				}
				case BusRd_:{
					break;
				}
				case BusRdX:{
					break;
				}
			}
			break;
		}
	}
}
void Core_main(int i)//change cpu_status, may change core_finish
{
	cpu_status[i].core_cycle++;

	if(cpu_status[i].is_waiting_bus)//waiting bus
	{
		cpu_status[i].idle_cycle++;
		return;
	}
	if(cpu_status[i].busy_remain_cycle)//doing calculate
	{
		cpu_status[i].busy_remain_cycle--;
		return;
	}
	//prepare to get a new ins
	uint32_t ins_type,ins_address;
	int fscanf_res=0;
	fscanf_res=fscanf(trace_fp[i],"%d %x",&ins_type,&ins_address);
	if(fscanf_res!=2)
	{
		core_finish[i]=1;
		cpu_status[i].core_cycle--;//no need this cycle
		continue;
	}
	switch (ins_type){
		case 0:{
			cpu_status[i].ls_ins++;
			uint32_t my_set=ins_address&set_mask;
			my_set>>=block_offset;
			int in_set_index=Cache_check_hit(cache_core[i][my_set],ins_address);
			cache_block* this_block;
			if(in_set_index<0)//miss
			{
				cpu_status[i].miss_count++;
				in_set_index=Cache_choose_replace(cache_core[i][my_set]);
				this_block=&cache_set[i][my_set][in_set_index];
				if(this_block->state!=Invalid){//kick out
					Bus_req_generate(i,ins_address,Flush);	
					this_block->state=Invalid;
				}
			}
			this_block=&cache_set[i][my_set][in_set_index];
			//check share
			Bus_share_signal=0;		
			uint8_t share_cpu_id[4]={0};
			if(Bus_check_share(ins_address,share_cpu_id))
				Bus_share_signal=1;
			
			int access_res=call_FSM(i,ins_address,this_block,PrRd,cpu_status[i].core_cycle);
			//0 for access private,1 access for shared
			if(access_res==0)
				cpu_status[i].access_private++;
			else if(access_res==1)
				cpu_status[i].access_shared++;
			break;
		}
		case 1:{
			cpu_status[i].ls_ins++;
			uint32_t my_set=ins_address&set_mask;
			my_set>>=block_offset;
			int in_set_index=Cache_check_hit(cache_core[i][my_set],ins_address);
			cache_block* this_block;
			if(in_set_index<0)//miss
			{
				cpu_status[i].miss_count++;
				in_set_index=Cache_choose_replace(cache_core[i][my_set]);
				this_block=&cache_set[i][my_set][in_set_index];
				if(this_block->state!=Invalid){//kick out
					Bus_req_generate(i,ins_address,Flush);	
					this_block->state=Invalid;
				}
			}
			this_block=&cache_set[i][my_set][in_set_index];
			//check share
			Bus_share_signal=0;		
			uint8_t share_cpu_id[4]={0};
			if(Bus_check_share(ins_address,share_cpu_id))
				Bus_share_signal=1;
			
			int access_res=call_FSM(i,ins_address,this_block,PrWr,cpu_status[i].core_cycle);
			//0 for access private,1 access for shared
			if(access_res==0)
				cpu_status[i].access_private++;
			else if(access_res==1)
				cpu_status[i].access_shared++;
			break;
		}
		case 2:{
			cpu_status[i].busy_remain_cycle=ins_address-1;
			cpu_status[i].compute_cycle+=ins_address;
			break;
		}
	}

}
void Bus_main()//change bus_status, may change cpu_status
{
	int i;
	node_t* bus_queue_ptr=&bus_status.bus_queue;
	if(is_empty(bus_queue_ptr)){
		for(i=0;i<4;i++)
			cpu_status[i].is_waiting_bus=0;
		return;
	}
	if(bus_status.head_remain_cycle==0){
		//start a new request
		bus_req * head_req_ptr=peek(bus_queue_ptr);
		bus_status.head_remain_cycle=head_req_ptr->estimate_cycle;
		bus_status.traffic_bytes+=head_req_ptr->traffic_bytes;
		if(head_req_ptr->request_type==BusRdX || head_req_ptr->request_type==BusUpd)
			bus_status.number_of_iu++;
		//handle snoop protocol
		uint32_t ins_address=head_req_ptr->address;
		//check share
		Bus_share_signal=0;		
		uint8_t share_cpu_id[4]={0};
		if(Bus_check_share(ins_address,share_cpu_id))
			Bus_share_signal=1;

		uint32_t my_set=ins_address&set_mask;
		my_set>>=block_offset;
		for(i=0;i<4;i++)
		{
			if(share_cpu_id[i] && head_req_ptr->cpu_id != i)//other cores involved
			{
				int in_set_index=Cache_check_hit(cache_core[i][my_set],ins_address);
				cache_block* this_block=&cache_set[i][my_set][in_set_index];
				call_FSM(i,ins_address,this_block,head_req_ptr->request_type,cpu_status[i].core_cycle);
			}
		}
		//check new status for bus waiting
		for(i=0;i<4;i++)
			cpu_status[i].is_waiting_bus=0;
		head_req_ptr=peek(bus_queue_ptr);
		while(head_req_ptr!=bus_queue_ptr)
		{
			cpu_status[head_req_ptr->cpu_id].is_waiting_bus=1;
			head_req_ptr=((node_t*)head_req_ptr)->next;
		}
		
	}
	else{
		bus_status.head_remain_cycle--;
		if(bus_status.head_remain_cycle==0)
			free(dequeue(bus_queue_ptr));
	}

}
void Bus_req_generate(int cpu_id,uint32_t address,int request_type)
{
	node_t* bus_queue_ptr=&bus_status.bus_queue;
	bus_req *new_req_ptr=malloc(sizeof(bus_req));
	new_req_ptr->cpu_id=cpu_id;
	new_req_ptr->request_type=request_type;
	new_req_ptr->address=address&no_offset_mask;//get rid of offset
	new_req_ptr->traffic_bytes=block_size;
	if(request_type==BusUpd)
		new_req_ptr->estimate_cycle=new_req_ptr->traffic_bytes/4*2;
	else if(request_type!=Nothing)
		new_req_ptr->estimate_cycle=100;
	enqueue(bus_queue_ptr,(node_t*)new_req_ptr);
}
int Bus_check_share(uint32_t address,uint8_t cpu[4])
{
	int i,j;
	int len=0;
	for(i=0;i<4;i++){
		cache_block* set_ptr=cache_core[i][(address&set_mask)>>block_offset];
		cpu[i]=0;
		for(j=0;j<associativity;j++)
		{
			if( address&tag_mask==set_ptr[j].tag && set_ptr[j].state!=Invalid)
			{
				cpu[i]=1;
				len++;
				break;
			}
		}
	}
	return len;
}
int Cache_check_hit(cache_block* set,uint32_t address)
{
	int i;
	uint32_t my_tag=address&tag_mask;
	for(i=0;i<associativity;i++)
	{
		if(set[i].tag==my_tag)//hit
			return i;
	}
	return -1;
}
int Cache_choose_replace(cache_block* set)
{
	//LRU
	int i,find_res;
	uint64_t min_timestamp=~0ull;
	for(i=0;i<associativity;i++)
	{
		if(set[i].state==Invalid)
			return i;
		if(set[i].timestamp<min_timestamp)
		{
			min_timestamp=set[i].timestamp;
			find_res=i;
		}
	}
	return find_res;
}



int main(int argc, char const *argv[])
{
	srand(time());

	if(argc!=6 || argc!=3)
	{
		printf("wrong arguments!\n");
		return 0;
	}
	//open trace files
	int i,j;
	for (i = 0; i < 4; ++i)
	{
		char buffer[64];
		sprintf(buffer,"%s_%d",argv[2],i);
		trace_fp[i]=fopen(buffer,"r");
	}
	//determine protocol
	if(strcmp("MESI",argv[1])==0)
		protocol=1;
	else if(strcmp("Dragon",argv[1])==0)
		protocol=2;
	else if(strcmp("MESI_adv",argv[1])==0)
		protocol=3;
	else
		exit(1);
	//handle settings
	if (argc==6)
	{
		cache_size=atoi(argv[3]);
		associativity=atoi(argv[4]);
		block_size=atoi(argv[5]);
		block_offset=0;
		uint32_t temp;
		temp=block_size;
		while(temp>>=1){
			block_offset++;
		}
		set_number=(cache_size/block_size)/associativity;
		temp=set_number;
		while(temp>>=1){
			set_index_len++;
		}
		no_offset_mask=(~0u)<<block_offset;
		tag_mask=no_offset_mask<<set_index_len;
		set_mask=no_offset_mask&(~tag_mask);		
	}
	//malloc things
	for (i = 0; i < 4; ++i)
	{
		cache_core[i]=malloc(set_number*sizeof(void*));
		for(j=0;j<set_number;j++)
			cache_core[i][j]=calloc(associativity*sizeof(cache_block));
	}
	queue_init(&bus_status.bus_queue);
	bus_status.head_remain_cycle=0;
	bus_status.traffic_bytes=0;
	bus_status.number_of_iu=0;

	//main loop
	while(!core_finish[0]||!core_finish[1]||!core_finish[2]||!core_finish[3])
	{

		//first bus main

		//then core
		int cpu_index[4]={0,1,2,3};
		int temp;
		i=4;
		while(i--)//random sort cpu for getting request
		{
			j=rand()%(i+1);
			temp=cpu_index[i];
			cpu_index[i]=cpu_index[j];
			cpu_index[j]=temp;
		}
		for(j=0;j<4;j++)//handle each core
		{
			i=cpu_index[j];
			if(core_finish[i])
				continue;
			Core_main(i);		
		}
	}
	//close files
	for (i = 0; i < 4; ++i)
		fclose(trace_fp[i]);
	//free malloc
	for (i = 0; i < 4; ++i)
	{
		for(j=0;j<set_number;j++)
			free(cache_core[i][j]);
		free(cache_core[i]);
	}
	//output result

	return 0;
}