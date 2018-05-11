module ALU(input logic[3:0] alu_in1, alu_in2, logic[2:0] alu_func , logic alu_op, output logic[3:0] alu_out, logic Carry_f, Zero_f);
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
				alu_out[0] = alu_in1[3];
				Carry_f = alu_in1[3]; 
			end
			else if (alu_func == 3'b111) begin
				alu_out = alu_in1 >> 1;
				alu_out[3] = alu_in1[0];
				Carry_f = alu_in1[3]; 
			end
			Zero_f = (alu_out == 4'b0000) ? 1:0;
		end
	end
endmodule // ALU8bit

// n_cs = not chip select, n_oe = not read mode, n_we = not write mode
module RAM256x8 (inout logic[7:0] ram_data, input logic[7:0] ram_address, input logic n_cs, n_oe, n_we, clk);
    logic[7:0] ram [255:0];
    assign data = (~n_cs & ~n_oe) ? ram_data[ram_address] : 8'bz;

    // always_latch begin
    always_ff @(posedge clk)
        if(~n_cs & n_oe & ~n_we)
            ram[ram_address] <= ram_data;
    end
endmodule

module mux2to1_4bit(input logic[3:0] mux4_in1, mux4_in2, logic mux4_sel, output logic[3:0] mux4_out);
    assign mux4_out = (mux4_sel) ? mux_in1 : mux4_in2;
endmodule

module mux4to1_8bit(input logic[7:0] mux8_in1, mux8_in2, mux8_in3,mux8_in4,logic[1:0] mux8_sel, output logic[7:0]mux8_out);
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


module adder(input logic[4:0] adder_in1, adder_in2, output logic[4:0] adder_out);
	adder_out = adder_in1 + adder_in2;
endmodule

module reg8x8(input logic regWrite,
					logic [2:0] reg_read_addr1, reg_read_addr2, reg_write_addr,
					logic [7:0] reg_d_in, 
			output	logic [7:0] reg_d_out1, reg_d_out2);
	
	logic [7:0] register[7:0];

	always_ff @(negedge regWrite) begin
		register[reg_write_addr] <= reg_d_in;
	end

	always_latch begin
		reg_d_out1 = register[reg_read_addr1];
		reg_d_out1 = register[reg_read_addr2];
	end
endmodule

module PC (input clk, PC_reset, logic [7:0] PC_in, output logic [7:0] PC_out);
	logic [7:0] PC_out;
	always @ (posedge clk | posedge reset) begin
		if(PC_reset)
			PC_out <= 8'b00000000;
		else
			PC_out <= PC_in;
	end
endmodule

// opcode 1+2
module ROM (input logic [7:0] rom_address, output logic [7:0] rom_data);
	logic[7:0] rom [0:255];
	int i;
	initial begin
		for(i=0; i<256; i = i + 1 ) begin
			rom[i] = 8'b0000_0000;
		end
	end

	// assign rom
	rom[0] = 8'b0000_0000; //nop
	rom[1] = 8'b0010_1111; //load reg 15
	rom[2] = 8'b0000_0001; //from mem(ram) 1 
 
	always_comb begin
		rom_data = rom[rom_address];
	end
endmodule

module jumpController(input logic[7:0] opcode1, logic Carry_f, Zero_f, output logic jumpCond);
	logic isJumpOpcode;
	logic jumpFunc[2:0]
	// always_ff @(posedge isJumpOpcode) begin
	always_comb begin
		isJumpOpcode = (opcode1[7:4] == 0'b0100) ? 1 : 0;
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
endmodule


// not finish
module Controller(input logic[7:0] opcode1,
				output logic n_cs, n_oe, n_we, regdest, alu_op, logic[1:0] mem_to_reg, logic[2:0] alu_func)
	
	always_comb begin
		// load reg with immediate value
		if(opcode1[7:4] == 4'b0001) begin
			n_cs = 0;	n_oe = 1;	n_we = 0;	alu_op = 0;
			mem_to_reg = 2'b00;
		end
		// load reg with memory content
		else if(opcode1[7:4] == 4'b0010) begin
			n_cs = 0;	n_oe = 1;	n_we = 0;	alu_op = 0;
			mem_to_reg = 2'b01;
		end
	end
endmodule

module CPU;
    // PC flipflop เก็บ counter ไว้ส่งให้ rom
	// ROM - Instruction memory / เรียกที่ testbench ทีเดียว
		//ส่ง opcode1, 2 ออกมา //opcode2 = rom[PC+1]
	// ต่อ opcode แบบมัดมือชก
		// controller opcode1[7:4]
		// branch-jump control opcode1[3:0], opcode2
	// ALU func=opcode[6:4] 
		// อาจจะมีโมดูลเรียกเลขจาก register หรือไม่ก็เรียกใน ALU เลย
		// reg1=opcode1[3:0] reg2=opcode2[7:4] reg3=opcode[3:0]
		// aluOp = (opcode1[7:6] == 2'b10) ? 1:0;
    // RAM
		// go to controller
		// n_cs, n_oe, n_we
		// n_cs = not chip select
			// 0 = store-load
			// 1 = jump, alu
		// n_oe = not read
			// 0 = jump, alu, store
			// 1 = load
		// n_we = not write
			// 0 = store-load
			// 1 = jump, alu
    // REG
		// regWrite
			// 0 = store, jump
			// 1 = alu, load
		// reg address
			// reg_read_addr1 = opcode1[3:0]
			// reg_read_addr2 = opcode2[7:4]
			// reg_write_addr 
				// regdest
				// 1 = load opcode1[3:0]
				// 0 = alu opcode2[3:0]
		// reg data
			// reg_d_out1 = reg[opcode1[3:0]]
			// reg_d_out2 = reg[opcode2[7:4]]
			// reg_d_in 
				// memToReg
				// 00 = load immediate opcode2
				// 01 = load ram[opcode2]
				// 10 = ALU output
	// JUMP
		// jumpcond = (opcode1[7:4] == 2'b0100)?1:0;
		// go to jump module
		// 0 = PC 1 = opcode2
    // ADDER PC = PC+1
    // MUX2to1 4 bit
	// MUX4to1 8 bit

endmodule