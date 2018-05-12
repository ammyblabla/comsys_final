module d_flipflop_8bit (input logic clk, logic[7:0] d, output logic[7:0] q);
  always_ff @(posedge clk, negedge clk) begin
	q <= d; 
  end
endmodule

module ALU(input wire[7:0] alu_in1, logic[7:0] alu_in2, logic[2:0] alu_func , logic alu_op, output logic[7:0] alu_out, logic Carry_f, Zero_f);

	initial begin
		$monitor("ALU time=%d alu_in1=%b alu_in2=%b alu_out=%b carry=%b zero=%b alu_func=%b alu_op=%b", $time, alu_in1, alu_in2, alu_out, Carry_f, Zero_f, alu_func, alu_op);	
	end

	always_comb begin
		if(alu_op) begin
			if(alu_func == 3'b000) begin
				alu_out = alu_in1 + alu_in2;
				Carry_f = (alu_in1[7] | alu_in2[7]) & (~alu_out[7]);
			end
			else if (alu_func == 3'b001) begin
			alu_out = alu_in1 - alu_in2;
			Carry_f = ((alu_in1[7] & alu_in2[7] & alu_out[7]) | (~alu_in1[7] & alu_in2[7] & (~alu_out[7]))) ? 1:0;
			end
			else if (alu_func == 3'b010) begin
				alu_out = alu_in1 & alu_in2;
				Carry_f = 0;
			end
			else if (alu_func == 3'b011) begin
				alu_out = alu_in1 | alu_in2;
				Carry_f = 0;
			end
			else if (alu_func == 3'b100) begin
				alu_out = alu_in1 ^ alu_in2;
				Carry_f = 0;
			end
			else if (alu_func == 3'b101) begin
				alu_out = ~alu_in1;
				Carry_f = 0;
			end
			else if (alu_func == 3'b110) begin
				alu_out = alu_in1 << 1;
				alu_out[0] = alu_in1[7];
				Carry_f = alu_in1[7]; 
			end
			else if (alu_func == 3'b111) begin
				alu_out = alu_in1 >> 1;
				alu_out[7] = alu_in1[0];
				Carry_f = alu_in1[7]; 
			end
			Zero_f = (alu_out == 4'b0000) ? 1:0;
		end
	end
endmodule // ALU8bit

// n_cs = not chip select, n_oe = not read mode, n_we = not write mode
module RAM256x8 (output wire[7:0] ram_data_out, input logic[7:0] ram_address,ram_data_in, input logic n_cs, n_oe, n_we, clk);
    logic[7:0] ram [255:0];

	initial begin	
		$monitor("ram time=%d ram_data_in=%b ram_data_out=%b ram_addr=%b n_cs=%b n_oe=%b n_we=%b", $time, ram_data_in,ram_data_out, ram_address, n_cs, n_oe, n_we);
	end

    assign ram_data_out= (~n_cs & ~n_oe) ? ram[ram_address] : 8'bz;

    // always_latch begin
    always_ff @(posedge clk) begin
        if(~n_cs & n_oe & ~n_we)
            ram[ram_address] <= ram_data_in;
		// else if (~n_cs & ~n_oe)
		// 	ram_data_out <= ram[ram_address];
    end
endmodule

module mux2to1_4bit(input logic[3:0] mux4_in1, mux4_in2, logic mux4_sel, output logic[3:0] mux4_out);
    assign mux4_out = (mux4_sel) ? mux4_in1 : mux4_in2;
	initial begin
		$monitor("mux214	time=%d mux4_in1=%b mux4_in2=%b mux4_sel=%b mux4_out=%b", $time, mux4_in1, mux4_in2, mux4_sel, mux4_out);
	end

endmodule

module mux4to1_8bit(input logic[7:0] mux8_in1, mux8_in3,mux8_in4,logic[1:0] mux8_sel, output logic[7:0]mux8_out, mux8_in2);
	
	initial begin
		$monitor("mux418	time=%d mux8_in1=%b  mux8_in2=%b mux8_in3=%b mux8_in4=%b mux8_sel=%b mux8_out=%b", $time, mux8_in1, mux8_in2, mux8_in3, mux8_in4, mux8_sel, mux8_out);
	end

	always_comb begin
		if(mux8_sel == 2'b00)
			mux8_out = mux8_in1;
		else if(mux8_sel == 2'b01)
			mux8_out = mux8_in2;
		else if(mux8_sel == 2'b10)
			mux8_out = mux8_in3;
		else if(mux8_sel == 2'b11)
			mux8_out = mux8_in4;
	end
endmodule

module reg16x8(input logic clk, regWrite,
					logic [3:0] reg_read_addr1, reg_read_addr2, reg_write_addr,
					logic [7:0] reg_d_in, 
			output	logic [7:0] reg_d_out2, logic [7:0] reg_d_out1);
	
	logic [7:0] register[15:0];

	initial begin
		$monitor("reg	time=%d clk=%b reg_read_addr1=%b reg_read_addr2=%b reg_write_addr=%b reg_d_in=%b reg_d_out1=%b reg_d_out2=%b", $time, clk,reg_read_addr1, reg_read_addr2, reg_write_addr, reg_d_in, reg_d_out1, reg_d_out2);
		// $monitor("reg time=%d write_result reg[%b]=%b reg_d_in=%b", $time, reg_write_addr, register[reg_write_addr], reg_d_in);
		// $monitor("reg time=%d read_result 1 reg[%b]=%b 2 reg[%b]=%b", $time, reg_d_out1, register[reg_d_out1], reg_d_out2, register[reg_d_out2]);
	end
	// assign reg_d_out1 = (~regWrite)?register[reg_read_addr1]:8'bz;
	// assign reg_d_out2 = (~regWrite)?register[reg_read_addr2]:8'bz;

	always_latch begin
		reg_d_out1 <= register[reg_read_addr1];
		reg_d_out2 <= register[reg_read_addr2];
	// end

	// always_ff @(negedge clk) begin
		if(clk) begin
			if(regWrite) register[reg_write_addr] <= reg_d_in;
		end
	end

endmodule

module PC(input logic[7:0] branch_addr, logic jumpCond, clk,reset ,output logic[7:0] PC_Out);
	logic[7:0] current_addr;
	logic[7:0] next_addr;

	initial begin
		$monitor("PC time=%d branch_addr=%b	jumpcond=%b	current_addr=%b	next_addr=%b	reset=%b", $time,branch_addr, jumpCond, current_addr, next_addr,reset);
	end

	always_comb begin
		if(reset) next_addr = 8'b0000_0000;
		else if (jumpCond == 0) next_addr = current_addr + 2;
		else if(jumpCond == 1) next_addr = branch_addr;
		PC_Out = current_addr;

		if(next_addr > 8'b1111_1111) next_addr = 0; 
	end
	d_flipflop_8bit ff(.clk(clk), .d(next_addr), .q(current_addr));
endmodule

module Controller(input logic[7:0] opcode1, input logic Carry_f, Zero_f,
				output logic n_cs, n_oe, n_we, regdest, alu_op, regWrite, jumpCond, logic[1:0] mem_to_reg, logic[2:0] alu_func);
	
	logic isJumpOpcode;
	logic jumpFunc[2:0];

	initial begin
		$monitor("controller	time=%d	opcode1=%b	Carry_f=%b	Zero_f=%b",$time,opcode1,Carry_f,Zero_f);
		$monitor("controller	time=%d	n_cs=%d	n_oe=%d	n_we=%d	regdest=%d	alu_op=%d	regWrite=%d	jumpCond=%d	mem_to_reg=%b	alu_func=%b",$time, n_cs, n_oe, n_we, regdest, alu_op, regWrite, jumpCond,  mem_to_reg, alu_func);
	end 

	always_comb begin
		// load reg with immediate value
		if(opcode1[7:4] == 4'b0001) begin
			n_cs = 1;	n_oe = 1;	n_we = 1;	alu_op = 0;
			mem_to_reg = 2'b00; jumpCond = 0;	regWrite = 1; regdest=1;
		end
		// read
		// load reg with memory content
		else if(opcode1[7:4] == 4'b0010) begin
			n_cs = 0;	n_oe = 0;	n_we = 1;	alu_op = 0;
			mem_to_reg = 2'b01; jumpCond = 0;	regWrite = 1; regdest=1;
		end
		// write
		// store
		else if(opcode1[7:4] == 4'b0011) begin
			n_cs = 0;	n_oe = 1;	n_we = 0;	alu_op = 0;
			mem_to_reg = 2'b11; jumpCond = 0;	regWrite = 0; regdest=1;
		end
		// ALU
		else if(opcode1[7]) begin
			n_cs = 1;	n_oe = 1;	n_we = 1;	alu_op = 1;
			mem_to_reg = 2'b10; jumpCond = 0;	regWrite = 1; regdest=0;
		end
		else begin
			isJumpOpcode = (opcode1[7:4] == 0'b0100) ? 1 : 0;
			n_cs = 1;	n_oe = 1;	n_we = 1;	alu_op = 0;
			mem_to_reg = 2'b11; regWrite = 0;
			if(isJumpOpcode) begin
				if(opcode1[2:0] == 3'b000)
					jumpCond = 1;
				else if(opcode1[1:0] == 2'b01) begin
					if(opcode1[2] == 0)
						jumpCond = Carry_f;
					else
						jumpCond = ~Carry_f;
				end
				else if(opcode1[1:0] == 2'b10) begin
					if(opcode1[2] == 0)
						jumpCond = Zero_f;
					else
						jumpCond = ~Zero_f;
				end 
			end
		end
	end
endmodule

module Datapath(input logic[7:0] opcode1, opcode2,
					logic [1:0] mem_to_reg,
					logic n_cs, n_oe, n_we, regdest, alu_op, jumpCond, clk, regWrite, reset,
				output logic[7:0] rom_address, logic Carry_f, Zero_f);

	logic [7:0] alu_in2, reg_d_in, reg_d_out2, alu_out,ram_data_in;
	// wire [7:0] ram_data_out;
	wire [7:0] ram_data_out;
	logic [2:0] alu_func;
	logic [3:0] reg_write_addr;
	// assign alu_in1 = reg_d_out1;
	assign alu_in2 = reg_d_out2;
	// assign ram_data = reg_d_out1;
	assign alu_func = opcode1[6:4];

	initial begin
		$monitor("datapath	time=%d	opcode1=%b	opcode2=%b rom_address=%b", $time, opcode1, opcode2, rom_address);
		$monitor("datapath	time=%d	n_cs=%d	n_oe=%d	n_we=%d	regdest=%d	alu_op=%d	regWrite=%d	jumpCond=%d	mem_to_reg=%b	alu_func=%b",$time, n_cs, n_oe, n_we, regdest, alu_op, regWrite, jumpCond,  mem_to_reg, alu_func);
		$monitor("datapath	time=%d	Carry_f=%d	Zero_f=%d reset=%d",$time,Carry_f,Zero_f,reset);
		$monitor("datapath	time=%d	ram_data_in=%b ram_data_out=%b reg_d_out2=%b	reg_d_in=%b	reg_write_addr=%b", $time, ram_data_in, ram_data_out, reg_d_out2, reg_d_in, reg_write_addr);
		$monitor("datapath	time=%d	alu_op=%b	alu_func=%b	alu_in2=%b	alu_out=%b", $time, alu_op, alu_func,  alu_in2, alu_out);
	end 

	PC pc (.branch_addr(opcode2), .jumpCond(jumpCond), .clk(clk),.reset(reset), .PC_Out(rom_address));
	mux2to1_4bit regDest_mux (.mux4_in1(opcode1[3:0]), .mux4_in2(opcode2[3:0]), .mux4_sel(regdest), .mux4_out(reg_write_addr));
	// mux select write to reg data
	mux4to1_8bit mem_to_reg_mux (.mux8_in1(opcode2), .mux8_in3(alu_out),.mux8_in4(8'b0000_0000),.mux8_sel(mem_to_reg), .mux8_out(reg_d_in), .mux8_in2(ram_data_out));
	reg16x8 registers (.clk(clk),.regWrite(regWrite),.reg_read_addr1(opcode1[3:0]), .reg_read_addr2(opcode2[7:4]), .reg_write_addr(reg_write_addr), .reg_d_in(reg_d_in), .reg_d_out1(ram_data_in), .reg_d_out2(reg_d_out2));
	// RAM256x8 ram (.ram_data(ram_data), .ram_address(opcode2), .n_cs(n_cs), .n_oe(n_oe), .n_we(n_we), .clk(clk));

	RAM256x8 ram (.ram_data_in(ram_data_in),.ram_data_out(ram_data_out), .ram_address(opcode2), .n_cs(n_cs), .n_oe(n_oe), .n_we(n_we), .clk(clk));


	ALU alu (.alu_in1(ram_data_in), .alu_in2(alu_in2), .alu_func(alu_func) , .alu_op(alu_op), .alu_out(alu_out), .Carry_f(Carry_f), .Zero_f(Zero_f));

endmodule

module CPU(input  logic[7:0] opcode1, opcode2,
				  logic clk, reset, 
		   output logic[7:0] rom_address);

	logic[1:0] mem_to_reg;
	logic[2:0] alu_func;
	logic n_cs, n_oe, n_we, regdest, alu_op, regWrite, jumpCond, Carry_f, Zero_f;

	Controller controller (.opcode1(opcode1), .Carry_f(Carry_f), .Zero_f(Zero_f), .n_cs(n_cs), .n_oe(n_oe), .n_we(n_we), .regdest(regdest), .alu_op(alu_op), .regWrite(regWrite), .jumpCond(jumpCond), .mem_to_reg(mem_to_reg), .alu_func(alu_func));

	Datapath datapath (.opcode1(opcode1), .opcode2(opcode2),.mem_to_reg(mem_to_reg), .n_cs(n_cs), .n_oe(n_oe), .n_we(n_we), .regdest(regdest), .alu_op(alu_op), .jumpCond(jumpCond), .clk(clk), .regWrite(regWrite), .reset(reset),.rom_address(rom_address), .Carry_f(Carry_f), .Zero_f(Zero_f));

	initial begin
		$monitor("CPU	time=%d	opcode1=%b	opcode2=%b	rom_address=%b",$time, opcode1, opcode2, rom_address);
	end
endmodule