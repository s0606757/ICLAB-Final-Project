module SNS(
	//input
	clk,
	rst_n,
	in_valid,
	in_number,
	in_dim_x,
	in_dim_y,
	in_dim_z,
	bomb,
	//output
	out_valid,
	out_sum
);
//input
input clk;
input rst_n;
input in_valid;
input [2:0]in_number;
input [2:0]in_dim_x;
input [2:0]in_dim_y;
input [2:0]in_dim_z;
input bomb;
//output
output reg out_valid;
output reg [13:0]out_sum;

//---------------------------------------------------------------------
//   VARIATIONS DECLARATION                         
//---------------------------------------------------------------------
reg [2:0] current_state,next_state;
reg[2:0] counter_x,counter_y,counter_z;
reg [511:0]block_position,bomb_position;
reg [2:0]in_dim_x_REG,in_dim_y_REG,in_dim_z_REG;
reg bomb_REG;
reg [2:0]in_number_REG;

reg[2:0]neighbor_Total;
reg[2:0]neighbor_counterX,neighbor_counterY,neighbor_counterZ;

reg block_position_delay_1cycle,block_position_delay_2cycle,block_position_delay_3cycle;
reg [2:0]neighbor_Total_delay_1cycle;
reg [13:0]Area;
reg[2:0]counter_area;
reg[2:0]counter_outputdata;
//---------------------------------------------------------------------
//   Memory VARIATIONS DECLARATION                         
//---------------------------------------------------------------------
reg CEN,WEN,OEN;
reg [8:0]  MEM_Add;
reg [11:0] MEM_IN;
wire [11:0]MEM_OUT;
reg  [11:0]MEM_OUT_reg;
//---------------------------------------------------------------------
//   PARAMETERS DECLARATION                         
//---------------------------------------------------------------------
parameter IDLE    			 =3'd0;
parameter DATA_INPUTING  	 =3'd1;
parameter ELIMINATING		 =3'd2;
parameter COMPUTING			 =3'd3;
parameter DATA_OUTPUTING	 =3'd4;

//---------------------------------------------------------------------
//   FSM CONTROL                       
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		current_state<=IDLE;
	else      
		current_state<=next_state;
end


always@(*)
begin
	case(current_state)
		IDLE:if(in_valid) 			next_state=DATA_INPUTING;
			 else 					next_state=IDLE;
		DATA_INPUTING:if(!in_valid)	next_state=ELIMINATING;
			 else 					next_state=DATA_INPUTING;
		ELIMINATING:if(counter_x==3'd7 && counter_y==3'd7 && counter_z==3'd7)  next_state=COMPUTING;
					else next_state=ELIMINATING;
		COMPUTING:if(counter_x==3'd7 && counter_y==3'd7 && counter_z==3'd7)  next_state=DATA_OUTPUTING;
					else next_state=COMPUTING;
		DATA_OUTPUTING:if(counter_outputdata==3)			next_state=IDLE;
						else next_state=DATA_OUTPUTING;
	default:next_state=IDLE;
	endcase
end
//---------------------------------------------------------------------
//   out_valid and out_sum             
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		out_valid<=1'b0;
	else if(current_state==DATA_OUTPUTING && counter_outputdata==3)
		out_valid<=1'b1;
	else
		out_valid<=1'b0;
end

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		out_sum<=14'b0;
	else if(current_state==DATA_OUTPUTING && counter_outputdata==3)
		out_sum<=Area;
	else
		out_sum<=14'b0;
end

//---------------------------------------------------------------------
//   counter_x   counter_y	counter_z                      
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		counter_outputdata<=3'd0;
	else if (current_state==IDLE)
		counter_outputdata<=3'd0;
	else if (current_state==DATA_OUTPUTING && counter_area==4)
		counter_outputdata<=counter_outputdata;	
	else if (current_state==DATA_OUTPUTING)
		counter_outputdata<=counter_outputdata+3'd1;
	else
		counter_outputdata<=counter_outputdata;
end


always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		counter_area<=3'd0;
	else if (current_state==IDLE)
		counter_area<=3'd0;
	else if(counter_outputdata==2)
		counter_area<=3'd0;
	else if (current_state==COMPUTING && counter_area==3)
		counter_area<=counter_area;	
	else if (current_state==COMPUTING)
		counter_area<=counter_area+3'd1;
	else
		counter_area<=counter_area;
end

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		counter_x<=3'd0;
	else if (current_state==ELIMINATING || current_state==COMPUTING)
		counter_x<=counter_x+3'd1;
	else
		counter_x<=counter_x;
end

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		counter_y<=3'd0;
	else if(counter_x==3'd7)
		counter_y<=counter_y+3'd1;
	else
		counter_y<=counter_y;
end

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		counter_z<=3'd0;
	else if(counter_y==3'd7 && counter_x==3'd7)
		counter_z<=counter_z+3'd1;
	else
		counter_z<=counter_z;
end
wire [2:0]counter_x_flow_add1,counter_x_flow_sub1;
wire [2:0]counter_y_flow_add1,counter_y_flow_sub1;
wire [2:0]counter_z_flow_add1,counter_z_flow_sub1;

assign counter_x_flow_add1=counter_x+3'd1;
assign counter_x_flow_sub1=counter_x+3'd7;

assign counter_y_flow_add1=counter_y+3'd1;
assign counter_y_flow_sub1=counter_y+3'd7;

assign counter_z_flow_add1=counter_z+3'd1;
assign counter_z_flow_sub1=counter_z+3'd7;


//---------------------------------------------------------------------
// Neighbor Counts                        
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		neighbor_counterX<=3'd0;
		neighbor_counterY<=3'd0;
		neighbor_counterZ<=3'd0;
	end
	else if (current_state==COMPUTING)
	begin
		case(counter_x)
		3'd0:neighbor_counterX <= block_position[ counter_z 	*64 +  counter_y 	* 8  +	counter_x+1	];
		3'd7:neighbor_counterX <= block_position[ counter_z 	*64 +  counter_y 	* 8  +	counter_x-1	];
		default:neighbor_counterX <= block_position[ counter_z 	*64 +  counter_y 	* 8  +	counter_x-1	] + block_position[ counter_z 	*64 +  counter_y 	* 8  +	counter_x+1	];
		endcase
		
		case(counter_y)
		3'd0:neighbor_counterY <= block_position[ counter_z 	*64 +  (counter_y+1) 	* 8  +	counter_x	];
		3'd7:neighbor_counterY <= block_position[ counter_z 	*64 +  (counter_y-1) 	* 8  +	counter_x	];
		default:neighbor_counterY <= block_position[ counter_z 	*64 +  (counter_y-1) 	* 8  +	counter_x	] + block_position[ counter_z 	*64 +  (counter_y+1) 	* 8  +	counter_x	];
		endcase
		
		case(counter_z)
		3'd0:neighbor_counterZ <= block_position[(counter_z+1) *64 +  counter_y 	* 8	 +	counter_x	];
		3'd7:neighbor_counterZ <= block_position[(counter_z-1) *64 +  counter_y 	* 8	 +	counter_x	];
		default:neighbor_counterZ <=block_position[(counter_z-1) *64 +  counter_y 	* 8	 +	counter_x	] + block_position[(counter_z+1) *64 +  counter_y 	* 8	 +	counter_x	];
		endcase
	end
	else
	begin
		neighbor_counterX<=3'd0;
		neighbor_counterY<=3'd0;
		neighbor_counterZ<=3'd0;
	end
end

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		neighbor_Total<=3'd0;
	else if (current_state==COMPUTING || CEN==0)
		neighbor_Total<=neighbor_counterX+neighbor_counterY+neighbor_counterZ;
	else
		neighbor_Total<=3'd0;
end


always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		neighbor_Total_delay_1cycle<=3'd0;
	else
		neighbor_Total_delay_1cycle<=neighbor_Total;
end

//---------------------------------------------------------------------
// Area Compute                        
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		Area<=14'd0;
	else if(current_state==IDLE)
		Area<=14'd0;
	else if(counter_area==3 && block_position_delay_3cycle==1'b1)
		Area<=Area+(3'd6-neighbor_Total_delay_1cycle)*MEM_OUT_reg;
	else
		Area<=Area;
end
//---------------------------------------------------------------------
// Positon                        
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		block_position<=512'd0;
	else if(current_state==IDLE)//reset block
		block_position<=512'd0;
	else if(current_state==DATA_INPUTING)
		block_position[in_dim_z_REG *64 +in_dim_y_REG * 8+in_dim_x_REG]<=1'b1;
	
	
	else if(current_state==ELIMINATING && bomb_position[counter_z *64 +counter_y * 8+counter_x]==1)
	begin
		block_position[ counter_z 		   *64 +  counter_y 		   * 8  +	counter_x			]<=1'b0;//bomb itself position
		 
		block_position[ counter_z 		   *64 +  counter_y 		   * 8  +	counter_x_flow_add1	]<=1'b0;//position_y and position_z are fixed tnen bombs position_x
		block_position[ counter_z 		   *64 +  counter_y 		   * 8	 +	counter_x_flow_sub1	]<=1'b0;//position_y and position_z are fixed tnen bombs position_x
		 
		block_position[ counter_z 		   *64 +  counter_y_flow_add1  * 8 +	counter_x			]<=1'b0;//position_x and position_z are fixed tnen bombs position_y
		block_position[ counter_z 		   *64 +  counter_y_flow_sub1  * 8 +	counter_x			]<=1'b0;//position_x and position_z are fixed tnen bombs position_y
				
		block_position[counter_z_flow_add1 *64 +  counter_y 		   * 8	 +	counter_x			]<=1'b0;//position_x and position_y are fixed tnen bombs position_z
		block_position[counter_z_flow_sub1 *64 +  counter_y 		   * 8	 +	counter_x			]<=1'b0;//position_x and position_y are fixed tnen bombs position_z
	end	
	
	
	
	
	else
		block_position<=block_position;
end

//---------------------------------------------------------------------
// Positon  1bit delay                      
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		block_position_delay_1cycle<=1'b0;
	else
		block_position_delay_1cycle<=block_position[ counter_z *64 +  counter_y * 8  +  counter_x ];
end

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		block_position_delay_2cycle<=1'b0;
	else
		block_position_delay_2cycle<=block_position_delay_1cycle;
end

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		block_position_delay_3cycle<=1'b0;
	else
		block_position_delay_3cycle<=block_position_delay_2cycle;
end

//---------------------------------------------------------------------
// Bomb                       
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		bomb_position<=512'd0;
	else if(current_state==IDLE)//reset bomb
		bomb_position<=512'd0;
	else if(current_state==DATA_INPUTING && bomb_REG==1)
		bomb_position[in_dim_z_REG *64 +in_dim_y_REG * 8+in_dim_x_REG]<=1'b1;
	else
		bomb_position<=bomb_position;
end
		
		

//---------------------------------------------------------------------
//    MEM_CEN control      
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		CEN<=1'b1;//standby:1
	else if(current_state==DATA_INPUTING || current_state==COMPUTING )
		CEN<=1'b0;//write or read:0
	else if(current_state==DATA_OUTPUTING)
		CEN<=1'b1;//write or read:0		
	else
		CEN<=1'b1;
end

//---------------------------------------------------------------------
//    MEM_WEN contro
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		WEN<=1'b1;//standby:X
	else if(current_state==DATA_INPUTING)
		WEN<=1'b0;//write:0
	else if(next_state==DATA_OUTPUTING)
		WEN<=1'b1;//read:1
	else
		WEN<=1'b1;
end

//---------------------------------------------------------------------
//    MEM_OEN control    
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		OEN<=1'b0;//standby:0
	else if(next_state==DATA_INPUTING)
		OEN<=1'b0;//read write:0
	else if(next_state==DATA_OUTPUTING)
		OEN<=1'b0;//read write:0
	else
		OEN<=1'b0;
end
//---------------------------------------------------------------------
//    MEM_Add control      
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n) 
		MEM_Add<=9'd0;
	else if (next_state==DATA_INPUTING || current_state==DATA_INPUTING)
		MEM_Add<=in_dim_z_REG *64 +in_dim_y_REG * 8+in_dim_x_REG;
	else if (current_state==COMPUTING)
		MEM_Add<=counter_z *64 +counter_y * 8 + counter_x;
	else
		MEM_Add<=MEM_Add;
end

//---------------------------------------------------------------------
//    MEM_IN control      
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n) 
		MEM_IN<=12'd0;
	else if(current_state==DATA_INPUTING)
		MEM_IN<={9'b0,in_number_REG};
	else
		MEM_IN<=MEM_IN;
end

//---------------------------------------------------------------------
//    MEM_OUT BUF       
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		MEM_OUT_reg<=12'd0;
	else
		MEM_OUT_reg<=MEM_OUT;
end


//---------------------------------------------------------------------
//    inclue Memory         
//---------------------------------------------------------------------
RA1SH SRAM12x512(
   .Q(MEM_OUT),
   .CLK(clk),
   .CEN(CEN),
   .WEN(WEN),
   .OEN(OEN),
   .A(MEM_Add),
   .D(MEM_IN)
);

//---------------------------------------------------------------------
//    in_dim_x  in_dim_y  in_dim_z  buffer        
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		in_dim_x_REG<=0;
		in_dim_y_REG<=0;
		in_dim_z_REG<=0;
	end
	else
	begin
		in_dim_x_REG<=in_dim_x;
		in_dim_y_REG<=in_dim_y;
		in_dim_z_REG<=in_dim_z;
	end
end

//---------------------------------------------------------------------
//    bomb  buffer        
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		bomb_REG<=0;
	else
		bomb_REG<=bomb;
end

//---------------------------------------------------------------------
//    in_number  buffer        
//---------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		in_number_REG<=0;
	else
		in_number_REG<=in_number;
end

endmodule
