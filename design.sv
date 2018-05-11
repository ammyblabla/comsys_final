module d_flipflop_4 (input logic[3:0] d, logic clk, res, output logic[3:0] d);
    always_ff@(posedge clk, negedge clk) begin
        if(res)
            q = 4'b0;
        else
            q <= d;
    end
endmodule

module reg8x8(input logic regWrite,
					logic [2:0] reg_read_addr1, reg_read_addr2, reg_write_addr,
					logic [7:0] d_in, 
			output	logic [7:0] d_out1, d_out2);
	
	logic [7:0] register[7:0];

	always_ff @(negedge regWrite) begin
		register[reg_write_addr] <= d_in;
	end

	always_latch begin
		d_out1 = register[reg_read_addr1];
		d_out2 = register[reg_read_addr2];
	end
endmodule

module counterIncrement(input logic[4:0] in1_counter, in2_counter, output logic[4:0] out_counter);
	out_counter = in1_counter + in2_counter;
endmodule

module Controller(input logic[7:0] opcode1, opcode2);
	logic [2:0] alu_func;
	logic [2:0] jump_func;
	logic [7:0] ram_address;
	always_comb begin
		if(opcode1[7] == 1) begin //ALU
			alu_func = opcode1[6:4];
			reg_read_addr1 = opcode1[3:0];
			reg_read_addr2 = opcode2[7:4];
			reg_write_addr = opcode2[3:0];
			reg_d_in = out;
			// regWrite = 1;
		end
		else if(~opcode1[7] & opcode1[6]) //JUMP
			jump_func = opcode1[6:4];
		else if(~opcode1[7] & (~opcode1[6])) begin
			if (opcode1[5:4] == 2'b01) begin //load immediate value to reg
				reg_write_addr = opcode1[3:0];
				reg_d_in = opcode2;
				// regWrite = 1;
				// n_oe = 1;
			end
			else if (opcode1[5:4] == 2'b10) begin //ram to reg
				ram_address = opcode2;
				ram_data = reg_d_in;
				reg_write_addr = opcode1[3:0];
				// regWrite = 1;
				// n_oe = 0;

			end
			else if (opcode1[5:4] == 2'b11) begin //move reg to ram
				reg_read_addr2 = opcode1[3:0];
				ram_data = reg_d_out2
				ram_address = opcode2;
				// regWrite = 1;
				// n_oe = 1;
			end

			// n_cs = 0;
			// n_we = ~n_oe;
		end
	end

	reg8x8 re0 (.regWrite(regWrite),.reg_read_addr1(reg_read_addr1), .reg_read_addr2(reg_read_addr2), .reg_write_addr(reg_write_addr),.d_in(reg_d_in),.d_out1(reg_d_out), .d_out2(reg_d_out2);
	ALU a0 (in1, in2, .alu_func(alu_func), .out(out));
	RAM256x8 ra0 (.data(ram_data), .address(ram_address), .n_cs(n_cs), .n_oe(n_oe), .n_we(n_we));
endmodule

module ALU(input logic[3:0] in1, in2, logic[2:0] alu_func, output logic[3:0] out);
	always_comb begin
		if(alu_func == 3'b000) begin
			out = in1 + in2;
			Carry_f = (in1[7] | in2[7]) & (~out[7]);
		end
		else if (alu_func == 3'b001) begin
		out = in1 - in2;
		Carry_f = ((in1[7] & in2[7] & out[7]) | (~in1[7] & in2[7] & (~out[7]))) ? 1:0;
		end
		else if (alu_func == 3'b010) begin
			out = in1 & in2;
			Carry_f = 0;
		end
		else if (alu_func == 3'b011) begin
			out = in1 | in2;
			Carry_f = 0;
		end
		else if (alu_func == 3'b100) begin
			out = in1 ^ in2;
			Carry_f = 0;
		end
		else if (alu_func == 3'b101) begin
			out = ~in1;
			Carry_f = 0;
		end
		else if (alu_func == 3'b110) begin
			out = in1 << 1;
			out[0] = in1[3];
			Carry_f = in1[3]; 
		end
		else if (alu_func == 3'b111) begin
			out = in1 >> 1;
			out[3] = in1[0];
			Carry_f = in1[3]; 
		end
		Zero_f = (out == 4'b0000) ? 1:0;

	end
endmodule // ALU8bit

module RAM256x8 (inout logic[7:0] data, input logic[7:0] address, input logic n_cs, n_oe, n_we);
    logic[7:0] ram [255:0];
    assign data = (~n_cs & ~n_oe) ? mem[address] : 8'bz;

    always_latch begin
        if(~n_cs & n_oe & ~n_we)
            ram[address] <= data;
    end
endmodule

// aka instruction memory
module ROM (input logic [3:0] address, output logic [7:0] data);
// Data Storage Array
logic [7:0] opcode1 [15:0] = {
	8'b0010_1011, // load reg 1011 with memory content
	8'b0100_1111, // mem = 79
	8'b0010_0110, // load reg 1011 with memory content
	8'b0111_1101, // mem = 125
	8'b0000_0111, // 
	8'b1111_1111, // 8
	8'b1110_1111, // 9
	8'b1111_0111, // A
	8'b1111_1100, // b
	8'b1011_1001, // C
	8'b1101_1110, // d
	8'b1111_1001, // E
	8'b1111_0001, // F
	8'b0001_1101, // load reg 1101 with imm
	8'b0011_1111, // imm = 31
	8'b0000_0110 // noop
};
// Memory Protocol Interface
always_comb begin
data = rom[address];
end
endmodule



// work as rom
module controller ()
	always_comb begin
		if(opcode1[7] == 1) begin //ALU
			alu_func = opcode1[6:4];
			reg_read_addr1 = opcode1[3:0];
			reg_read_addr2 = opcode2[7:4];
			reg_write_addr = opcode2[3:0];
			reg_d_in = out;
			regWrite = 1;
		end
		else if(~opcode1[7] & opcode1[6]) //JUMP
			jump_func = opcode1[6:4];
		else if(~opcode1[7] & (~opcode1[6])) begin
			if (opcode1[5:4] == 2'b01) begin //load immediate value to reg
				reg_write_addr = opcode1[3:0];
				reg_d_in = opcode2;
				regWrite = 1;
				n_oe = 1;
			end
			else if (opcode1[5:4] == 2'b10) begin //ram to reg
				ram_address = opcode2;
				ram_data = reg_d_in;
				reg_write_addr = opcode1[3:0];
				regWrite = 1;
				n_oe = 0;

			end
			else if (opcode1[5:4] == 2'b11) begin //move reg to ram
				reg_read_addr2 = opcode1[3:0];
				ram_data = reg_d_out2
				ram_address = opcode2;
				regWrite = 1;
				n_oe = 1;
			end

			n_cs = 0;
			n_we = ~n_oe;
		end
endmodule


// module jump(input logic[2:0] func, logic[7:0] pc_addr, output logic pc_addr_out);
//     logic[7:0] rom [15:0] = {}
    
//     logic[7:0] PC;
//     always_comb begin
//         if(func == 3'b000)
            
//     end
    
// endmodule