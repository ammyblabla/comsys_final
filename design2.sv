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
    assign mux4_out = (mux4_sel) ? mux4_in1 : mux4_in2;
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

module PC(input logic[7:0] branch_addr, logic jumpCond, clk, output int PC_Out);
  int current_addr;
  int next_addr;
  always_comb begin
    if (jumpCond == 0) next_addr = current_addr + 1;
    else if(jumpCond == 1) next_addr = branch_addr;
 	PC_Out = current_addr;
  end
  d_flipflop_7bit ff(.clk(clk), .d(next_addr), .q(current_addr));
endmodule


// opcode 1+2
module ROM (input logic [7:0] rom_address, output logic [7:0] rom_data1, rom_data2);
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
		rom_data1 = rom[rom_address];
		rom_data2 = (rom_address == 255) ? 8'b0000_0000:rom[rom_address+1];
	end
endmodule

module Controller(input logic[7:0] opcode1, input logic Carry_f, Zero_f,
				output logic n_cs, n_oe, n_we, regdest, alu_op, regWrite, jumpCond, logic[1:0] mem_to_reg, logic[2:0] alu_func)
	
	logic isJumpOpcode;
	logic jumpFunc[2:0]

	always_comb begin
		// load reg with immediate value
		if(opcode1[7:4] == 4'b0001) begin
			n_cs = 1;	n_oe = 1;	n_we = 1;	alu_op = 0;
			mem_to_reg = 2'b00; jumpCond = 0;	regWrite1 = 1;
		end
		// read
		// load reg with memory content
		else if(opcode1[7:4] == 4'b0010) begin
			n_cs = 0;	n_oe = 0;	n_we = 1;	alu_op = 0;
			mem_to_reg = 2'b01; jumpCond = 0;	regWrite = 1;
		end
		// write
		// store
		else if(opcode1[7:4] == 4'b0011) begin
			n_cs = 0;	n_oe = 1;	n_we = 0;	alu_op = 0;
			mem_to_reg = 2'b11; jumpCond = 0;	regWrite = 0;
		end
		// ALU
		else if(opcode1[7]) begin
			n_cs = 1;	n_oe = 1;	n_we = 1;	alu_op = 1;
			mem_to_reg = 2'b10; jumpCond = 0;	regWrite = 1;
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

module datapath(inout logic[7:0] ram_data,
				input logic[7:0] opcode1, opcode2,
					logic [1:0] mem_to_reg,
					logic  Carry_f, Zero_f, n_cs, n_oe, n_we, regdest, alu_op, jumpCond, clk, regWrite, reset
				output logic[7:0] rom_address);

	logic [3:0] alu_in1;
	logic [3:0] alu_in2;
	logic [7:0] adder_out;
	logic [7:0] reg_d_in;
	logic [2:0] reg_write_addr;
	logic [7:0] reg_d_out1;
	logic [7:0] reg_d_out2;

	assign alu_in1 = reg_d_out1;
	assign alu_in2 = reg_d_out2;
	assign din_1_load = ram_data[opcode2];
	assign ram_data = reg_d_out1;

	PC pc (.branch_addr(opcode2), .jumpCond(jumpCond), .clk(clk), .PC_out(rom_address));
	mux2to1_4bit regDest_mux (.mux4_in1(opcode1[3:0]), .mux4_in2(opcode2[3:0]), .mux4_sel(regdest), .mux4_out(reg_write_addr));
	mux4to1_8bit mem_to_reg_mux (.mux8_in1(opcode2), mux8_in2(ram_data), .mux8_in3(alu_out),.mux8_in4(8'b0000_0000),.mem_to_reg(mem_to_re), .mux8_out(reg_d_in));
	reg8x8 registers (.regWrite(regWrite),.reg_read_addr1(opcode1[3:0]), .reg_read_addr2(opcode2[7:4]), .reg_write_addr(reg_write_addr), reg_d_in(reg_d_in), .reg_d_out1(reg_d_out1), .reg_d_out2(reg_d_out2));
	RAM256x8 ram (.ram_data(reg_d_out1), .ram_address(opcode2), .n_cs(n_cs), .n_oe(n_oe), .n_we(n_we), .clk(clk));
	ALU alu (.alu_in1(alu_in1), .alu_in2(alu_in2), .alu_func(alu_func) , .alu_op(alu_op), .alu_out(alu_out), .Carry_f(Carry_f), .Zero_f(Zero_f));

endmodule


module CPU(input clk, reset);


endmodule