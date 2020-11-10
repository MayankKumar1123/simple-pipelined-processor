//***************************************INSTRUCTION MEMORY(USED BY IF)***************************************
module Instruction_Memory(
    input [7:0] PC,
    input reset,
    output [7:0] Instruction_Code
);
reg [7:0] Mem [80:0];  //Byte addresssable memory 36 locations

//For normal memory read we use the following statement 
assign Instruction_Code= Mem[PC];

//Handling reset condition
always @(reset)
    begin
    
    if(reset == 0)//reset logic is 0
        begin
        
            //Testing Instructions
            // Mem[0]   = 8'b01000001; // add r0 = r0 + r1
            // Mem[1]   = 8'b01000010; // add r0 = r0 + r2
            // Mem[2]   = 8'b11000101; //jmp to 5
            // Mem[3]   = 8'b00100000; //mov r4 = r0
            // Mem[4]   = 8'b01111010; //add r7 = r7 + r2
            // Mem[5]   = 8'b00110101; //mov r6 = r5
            // Mem[6]   = 8'b01110101; //add r6 = r6 + r5
            // Mem[7]   = 8'b11111111; //jump to 63
            // Mem[63]  = 8'b11000101; //jump to 64+5, do NOT execute b10000000
            // Mem[64]  = 8'b01000111; //add r0 = r0 + r7, NOT TO BE EXECUTED
            // Mem[65]  = 8'b01000000; //add r0 = r0 + r0, NOT TO BE EXECUTED
            // Mem[66]  = 8'b01000001; //add r0 = r0 + r1, NOT TO BE EXECUTED
            // Mem[67]  = 8'b01000010; //add r0 = r0 + r2, NOT TO BE EXECUTED
            // Mem[68]  = 8'b01000011; //add r0 = r0 + r3, NOT TO BE EXECUTED
            // Mem[69]  = 8'b01001010; //add r1 = r1 + r2
            // Mem[70]  = 8'b01001010; //add r1 = r1 + r2
            // Mem[71]  = 8'b01001010; //add r1 = r1 + r2
            
            // Mem[0]   = 8'b01010001; // add r2 = r2 + r1
            // Mem[1]   = 8'b01011010; // add r3 = r3 + r2
            // Mem[2]   = 8'b11000101; //jmp to 5
            // Mem[3]   = 8'b00100000; //mov r4 = r0
            // Mem[4]   = 8'b01111010; //add r7 = r7 + r2
            // Mem[5]   = 8'b00110101; //mov r6 = r5
            // Mem[6]   = 8'b01111101; //add r7 = r7 + r5
            
            
            //Instructions given by Assignment
            Mem[0] = 8'b00110010; //mov r6, r2
            Mem[1] = 8'b01010110; //add r2, r6
            Mem[2] = 8'b01100010; //add r4, r2
            Mem[3] = 8'b11000101; //j L1(or 5)
            Mem[4] = 8'b00110100; //mov r6, r4
            Mem[5] = 8'b01110100; //add r6, r4
            //Mem[6] = 8'b11000000; //j 0
            
        end
    end
endmodule

//***************************************PROGRAM COUNTER***************************************
module ProgramCounter(
input [7:0] PC_in, 
input clk,
input reset,
output reg [7:0] PC_out
);

    always@(posedge clk, negedge reset)
        begin
        if(reset == 0)
            PC_out = 0;
        else
            PC_out = PC_in;
        end



endmodule

//***************************************REGISTER FILE***************************************
module Register_File(
    input [2:0] Read_Reg_Num_1,
    input [2:0] Read_Reg_Num_2,
    input [2:0] Write_Reg_Num,
    input [7:0] Write_Data,
    input RegWrite,
    input reset,
    
    output [7:0] Read_Data_1,
    output [7:0] Read_Data_2
);

    reg [7:0] RegMemory [7:0];

    //Handling reset condition
    always @(negedge reset)
        begin
        
        if(reset == 0)//reset logic is 0
            begin 
            RegMemory[0] = 0;
            RegMemory[1] = 1;
            RegMemory[2] = 2;
            RegMemory[3] = 3;
            RegMemory[4] = 4;
            RegMemory[5] = 5;
            RegMemory[6] = 6;
            RegMemory[7] = 7;
            end
        end
    
    assign Read_Data_1 = RegMemory[Read_Reg_Num_1];  
    assign Read_Data_2 = RegMemory[Read_Reg_Num_2];  
        
        
    always @(RegWrite, Write_Reg_Num, Write_Data)
        begin
        if(RegWrite == 1)
            RegMemory[Write_Reg_Num] = Write_Data;  
        
        end

endmodule

//***************************************ALU***************************************
module ALU(
	input signed [7:0]A,
	input signed [7:0]B,
	input control,
	output reg zero_val,
	output reg signed [7:0]out
	);
    
	always@(A or B or control)
	begin
	case(control)
    
        0: out = A;
        1: out = A+B;
        
	endcase
    end
    
    
    always@(out)
    begin
        if(out == 0)
            zero_val = 1;
        else
            zero_val = 0;
    end
endmodule


//***************************************8 BIT MUX***************************************
module mux_2x1(
  input [7:0] in1,in2,
  input select,
  output reg [7:0] out);
  
  always@(in1, in2, select)
  begin
    if(select == 1'b0)
      out = in1;
    else
      out = in2;
  end
endmodule

//***************************************PC ADDER***************************************
module PCAdder(input [7:0] PC,
                output [7:0] PCAdd);

assign PCAdd = PC + 1;

endmodule

//***************************************PC APPENDER***************************************
module PCAppender(input [7:0] PC,
                input [5:0] PC_append_val,
                output [7:0] PC_append);

assign PC_append = {PC[7:6], PC_append_val};

endmodule

//***************************************CONTROL UNIT***************************************
module ControlUnit(input [1:0] OpCode, 
output ALUOp, 
//output jmp,
output RegWrite
);

assign ALUOp = OpCode[0];
assign RegWrite = !OpCode[1];
//assign jmp = OpCode[0] & OpCode[1];
endmodule

//***************************************INSTRUCTION DECODE***************************************
module Instruction_Decode(input [7:0] Ins_code, 
output [2:0] ID_Rd, 
output [2:0] ID_Rs, 
output [5:0] ID_jmp_imm, 
output [1:0] ID_OpCode);

assign ID_OpCode = Ins_code[7:6];
assign ID_Rd = Ins_code[5:3];
assign ID_Rs = Ins_code[2:0];
assign ID_jmp_imm = Ins_code[5:0];

endmodule


//***************************************FORWARDING UNIT***************************************
module ForwardUnit(
input [2:0] Rin1,
input [2:0] Rin2, 
input [2:0] RDest, 
input PL_EX_WB_RegWrite, 
output reg forwardMuxSel1,
output reg forwardMuxSel2
);
    always@(*)
        begin
        if((Rin1 == RDest) && (PL_EX_WB_RegWrite == 1))
            forwardMuxSel1= 1;//if Rs register and output register of EX are same,then forawrding is required
        else
            forwardMuxSel1= 0;
        if((Rin2 == RDest) && (PL_EX_WB_RegWrite == 1))
            forwardMuxSel2= 1;//if Rd register and output register of EX are same, then forawrding is required
        else
            forwardMuxSel2= 0;
        end
        
        
endmodule



//***************************************MAIN PROCESSOR IMPLEMENTATION***************************************
module processor(
    input clk,
    input reset
);

    //Variable definitions

    //Pipeline registers are named as PL_STAGE1_STAGE2_NAME, 
    //STAGE1, STAGE2 refer to the two stages the pipeline is in between and NAME refers to what value it should contain
    
    //In case of connecting wires, they are named as STAGE_NAME
    
    //For the PC
    wire [7:0] PC_Addr;
    wire [7:0] PC_Mux_out, PC_add, PC_append;

    wire [7:0] Ins_Code;//Output code directly from Instruction Memory
    
    //IF ---> ID

    reg [7:0] PL_IF_ID_PC_1;//PC+1 value stored
    reg [7:0] PL_IF_ID_Ins_code;//Code stored in the IF/ID pipeline register
    
    //In ID stage, there will be a Rd, Rs, imm(for jump) and control signals generated.
    wire [2:0] ID_Rs, ID_Rd;
    wire [5:0] ID_jmp_imm;
    wire [1:0] ID_OpCode;
    
    wire [7:0] ID_RegFile_Rs, ID_RegFile_Rd;//Outputs of the register file 
    
    //These are generated immediately by the control unit, and have to be stored to move it forward
    wire ALUOp;
    wire RegWrite;
    
    
    //ID ---> EX
    
    reg PL_ID_EX_ALUOp;
    reg PL_ID_EX_RegWrite;
    reg [2:0] PL_ID_EX_Rs;
    reg [2:0] PL_ID_EX_Rd;
    reg [7:0] PL_ID_EX_RegFile_Rs;
    reg [7:0] PL_ID_EX_RegFile_Rd;
    
    wire [7:0] EX_ALUin1;
    wire [7:0] EX_ALUin2;
    
    wire [7:0] EX_ALU_Out;
    wire ALU_Zero_val;
    
    //For forawrding unit, mux selects
    wire FwdSel1, FwdSel2;
    
    //EX ---> WB
    reg PL_EX_WB_RegWrite;
    reg [2:0] PL_EX_WB_Rd;
    reg [7:0] PL_EX_WB_ALU_Out;
    
    //Defining DataBlocks here
    
    
    //--------IF STAGE
    //Get the address of the next instruction to be fetched
    ProgramCounter PC(PC_add,clk,reset, PC_Addr);
    PCAdder PC_PC1(PC_Mux_out, PC_add);
    PCAppender PC_Jmp(PL_IF_ID_PC_1, ID_jmp_imm, PC_append);
    mux_2x1 PC_Mux(PC_append, PC_Addr, RegWrite, PC_Mux_out);
    
    
    //Put it in IM, to get code
    Instruction_Memory IM(PC_Mux_out, reset, Ins_Code);
    
    
    //--------ID STAGE
    Instruction_Decode ID(PL_IF_ID_Ins_code, ID_Rd, ID_Rs, ID_jmp_imm, ID_OpCode);
    ControlUnit CU(ID_OpCode, ALUOp, RegWrite);
    Register_File RF(ID_Rs, ID_Rd, PL_EX_WB_Rd, PL_EX_WB_ALU_Out, PL_EX_WB_RegWrite, reset,ID_RegFile_Rs, ID_RegFile_Rd );


    //--------EX STAGE
    ForwardUnit Forward(PL_ID_EX_Rs, PL_ID_EX_Rd, PL_EX_WB_Rd,PL_EX_WB_RegWrite, FwdSel1, FwdSel2);
    mux_2x1 FwdMux1(PL_ID_EX_RegFile_Rs,PL_EX_WB_ALU_Out,FwdSel1,EX_ALUin1);
    mux_2x1 FwdMux2(PL_ID_EX_RegFile_Rd,PL_EX_WB_ALU_Out,FwdSel2,EX_ALUin2);
    ALU EX(EX_ALUin1, EX_ALUin2, PL_ID_EX_ALUOp, ALU_Zero_val, EX_ALU_Out);
    
    
    //--------WB STAGE
    //No datablock
    
    
    
    //Order is backwards to ensure correct values go to the next stage
    //---------------EX_WB PIPELINE
    always@(posedge clk, negedge reset)
    begin
        if(reset == 0)
            begin
            PL_EX_WB_RegWrite = 0;
            PL_EX_WB_ALU_Out = 0;
            PL_EX_WB_Rd = 0;
            //Assign everything zero here
            end
        else
            begin
            PL_EX_WB_RegWrite = PL_ID_EX_RegWrite;
            PL_EX_WB_ALU_Out = EX_ALU_Out;
            PL_EX_WB_Rd = PL_ID_EX_Rd;
            end
    end
    
    
    //---------------ID_EX PIPELINE
    always@(posedge clk, negedge reset)
    begin
        if(reset == 0)
            begin
            PL_ID_EX_ALUOp = 0;
            PL_ID_EX_RegWrite = 0;
            PL_ID_EX_Rs = 0;
            PL_ID_EX_Rd = 0;
            PL_ID_EX_RegFile_Rs = 0;
            PL_ID_EX_RegFile_Rd = 0;
            //Assign everything zero here
            end
        else
            begin
            PL_ID_EX_ALUOp = ALUOp;
            PL_ID_EX_RegWrite = RegWrite;
            PL_ID_EX_Rs = ID_Rs;
            PL_ID_EX_Rd = ID_Rd;
            PL_ID_EX_RegFile_Rs = ID_RegFile_Rs;
            PL_ID_EX_RegFile_Rd = ID_RegFile_Rd;
            end
    end


    //---------------IF_ID PIPELINE
    always@(posedge clk, negedge reset)
    begin
        if(reset == 0)
            begin
            //Assign everything zero here
            PL_IF_ID_PC_1 = 0;
            PL_IF_ID_Ins_code = 0;
            
            end
        else
            begin
            PL_IF_ID_Ins_code = Ins_Code;
            PL_IF_ID_PC_1 = PC_add;
            end
    end
    
endmodule




module tb_processor;
reg clk;
reg reset;


    processor uut(clk, reset);
    //Get the clock up
    initial
    begin  
        clk = 1;
        forever #5 clk = ~clk;
    end

    
    //reset a few times 
    initial
    begin  
             reset = 1;
        #1   reset = 0;
        #18  reset = 1;
        #400 reset = 0;
        #10  reset = 1;
    end


endmodule