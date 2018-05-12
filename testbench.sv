module testbench(output logic[7:0] opcode1, opcode2, logic reset);    
    
	initial begin
		   reset = 0; opcode1 = 8'b0000_0000; opcode2 = 8'b0000_0000; 
		#1 opcode1 = 8'b0001_0000; opcode1 = 8'b1111_1111;
	end

endmodule

module top;
	logic[7:0] opcode1, opcode2;
	logic clk, reset;
	logic[7:0] rom_address;
	initial begin
		$dumpfile("dump.vcd");
		$dumpvars(1);
	end

	initial begin
		clk = 0;
		forever #1 clk = ~clk;
	end

	testbench t (.opcode1(opcode1), .opcode2(opcode2), .reset(reset));
	CPU cpu (.opcode1(opcode1), .opcode2(opcode2), .clk(clk), .reset(reset), .rom_address(rom_address));
endmodule


// // opcode 1+2
// module ROM (input logic [7:0] rom_address, output logic [7:0] rom_data1, rom_data2);
// 	logic[7:0] rom [0:255];
// 	int i;
// 	initial begin
// 		for(i=0; i<256; i = i + 1 ) begin
// 			rom[i] = 8'b0000_0000;
// 		end
// 	end

// 	// assign rom
// 	rom[0] = 8'b0000_0000; //nop
// 	rom[1] = 8'b0010_1111; //load reg 15
// 	rom[2] = 8'b0000_0001; //from mem(ram) 1 
 
// 	always_comb begin
// 		rom_data1 = rom[rom_address];
// 		rom_data2 = (rom_address == 255) ? 8'b0000_0000:rom[rom_address+1];
// 	end
// endmodule